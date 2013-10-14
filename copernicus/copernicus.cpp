/* 
 * File:   CopernicusGPS.cpp
 * Author: tbabb
 * 
 * Created on October 5, 2013, 10:35 PM
 */

#include <algorithm>

#include "copernicus.h"
#include "chunk.h"
#include "Arduino.h"

#define SAVE_BYTES(dst, buf, n) \
        if (readDataBytes(buf, n) != (n)) return false; \
        copy_network_order(dst, buf);

/***************************
 * structors               *
 ***************************/


/**
 * Construct a new `CopernicusGPS` object.
 * 
 * @param serial_num Arduino serial stream to monitor. 0 is `Serial`, 1 is 
 * `Serial1`, etc.
 */
CopernicusGPS::CopernicusGPS(int serial_num):
        m_n_listeners(0) {
    // ifdefs mirrored from HardwareSerial.h
    switch (serial_num) {
#ifdef UBRR1H
        case 1: m_serial = &Serial1; break;
#endif
#ifdef UBRR2H
        case 2: m_serial = &Serial2; break;
#endif
#ifdef UBRR3H
        case 3: m_serial = &Serial3; break;
#endif
        default: 
#if defined(UBRRH) || defined(UBRR0H)
            m_serial = &Serial;
#else
            m_serial = NULL;
#endif
    }
    if (m_serial != NULL) m_serial->begin(TSIP_BAUD_RATE);
}

/***************************
 * i/o                     *
 ***************************/

/**
 * Begin a TSIP command by sending the header bytes for the given
 * command type.
 * @param cmd Command to begin.
 */
void CopernicusGPS::beginCommand(CommandID cmd) {
    m_serial->write((uint8_t)CTRL_DLE);
    m_serial->write((uint8_t)cmd);
}

/**
 * End a command by sending the end-of-transmission byte sequence.
 */
void CopernicusGPS::endCommand() {
    m_serial->write((uint8_t)CTRL_DLE);
    m_serial->write((uint8_t)CTRL_ETX);
}

/**
 * Read data bytes from a TSIP report packet, unpacking any escape 
 * sequences in the process, placing `n` decoded bytes into `dst`. Blocks until
 * `n` bytes are decoded, or the end of the packet is reached (in which case
 * the two end-of-packet bytes will be consumed).
 * @param dst Destination buffer.
 * @param n Number of decoded bytes to read.
 * @return Number of bytes actually written to `dst`.
 */
int CopernicusGPS::readDataBytes(uint8_t *dst, int n) {
    for (int i = 0; i < n; i++, dst++) {
        blockForData();
        int read = m_serial->read();
        if (read == CTRL_DLE) {
            int peek = m_serial->peek();
            if (peek == -1) { 
                blockForData();
                peek = m_serial->peek();
            }
            if (peek == CTRL_DLE) {
                m_serial->read(); // advance the stream
            } else if (peek == CTRL_ETX) {
                // end of packet.
                m_serial->read(); // consume the ETX byte
                return i;
            }
        }
        *dst = read;
    }
    return n;
}

/**
 * Encode `n` data bytes as part of a TSIP command packet and send them to the 
 * gps module. Must be called only if a command has been opened with a call to 
 * `beginCommand()`, and may be called multiple times before a call to `endCommand()`. 
 * @param bytes Data bytes to encode and send.
 * @param n Number of bytes to encode.
 */
void CopernicusGPS::writeDataBytes(const uint8_t* bytes, int n) {
    for (int i = 0; i < n; i++) {
        m_serial->write(bytes[i]);
        if (bytes[i] == CTRL_DLE) {
            // avoid ambiguity with "end txmission" byte sequence
            m_serial->write(bytes[i]);
        }
    }
}

/**
 * Consume bytes from the serial input until an end-of-packet is
 * reached, stopping early if and only if no data is available and `block` is 
 * `false`. 
 * 
 * If this function returns `true`, the next data in the stream are expected to 
 * be the header of the next packet. To ensure correctness, an even number of 
 * `DLE` (0x10) bytes should have been consumed since the start of the current 
 * packet.
 * 
 * If the flush terminates early due to unavailable data, then the 
 * stream will be left in such a state that a later call to `flushToNextPacket()`
 * or `processOnePacket()` will behave as expected.
 * 
 * @param block Whether to wait for the complete packet to arrive.
 * @return True if the current packet was completely consumed/flushed.
 */
bool CopernicusGPS::flushToNextPacket(bool block) {
    while (true) {
        if (m_serial->available() <= 0) {
            if (block) blockForData();
            else return false;
        }
        int b = m_serial->read();
        if (b != CTRL_DLE) continue;
        if (m_serial->available() <= 0) blockForData();
        b = m_serial->read();
        if (b == CTRL_ETX) return true;
    }
}

/**
 * Process one TSIP packet from the stream, returning the ID of the packet
 * processed. If `block` is `false`, this function will return `RPT_NONE` if
 * no packet data was available. Otherwise, a valid packet ID or `RPT_ERROR` will
 * be returned.
 * 
 * Must be called regularly or in response to serial events. Example usage:
 *      
 *      // flush packets in a tight loop:
 *      while (gps.processOnePacket(false) != RPT_NONE) {}
 * 
 * Event handling:
 * 
 *      ReportType evt;
 *      while ((evt = gps.processOnePacket(false)) != RPT_NONE) {
 *          switch (evt) {
 *              // respond to updates
 *          }
 *      }
 * 
 * @param block If `true`, will always wait for a complete packet to arrive.
 * @return The report ID of the processed packet, or `RPT_NONE` if `block`
 * is `false` and no data was available.
 */
ReportType CopernicusGPS::processOnePacket(bool block) {
    implProcessOnePacket(block, RPT_NONE);
}

/**
 * Process packets/input until the header of a packet with type `type` is 
 * encountered, at which point the stream will be left for the caller to process.
 * The caller should fully consume the packet, including the end-of-packet bytes.
 * 
 * @param type Type of packet to wait for.
 */
void CopernicusGPS::waitForPacket(ReportType type) {
    while (implProcessOnePacket(false, type) != type) {}
}

/**
 * Consume the two terminating bytes of a TSIP packet, which
 * should be `0x10 0x03`. Return false if the expected
 * bytes were not found.
 */
bool CopernicusGPS::endReport() {
    blockForData();
    if (m_serial->read() != CTRL_DLE) return false;
    blockForData();
    if (m_serial->read() != CTRL_ETX) return false;
    return true;
}

/***********************
 * Commands            *
 ***********************/

/**
 * Set the format of position, velocity, and altitude fixes.
 * PPS settings and GPS time format may also be set with this command.
 * 
 * To leave a fix mode unchanged, pass `RPT_NONE`. Other mode settings
 * have `NOCHANGE` constants which will preserve the current settings.
 * 
 * @param pos_fixmode New position fix format. Any of the `RPT_FIX_POS_*` constants, or `RPT_NONE`.
 * @param vel_fixmode New velocity fix format. Any of the `RPT_FIX_VEL_*` constants, or `RPT_NONE`.
 * @param alt New altitude format.
 * @param pps New PPS setting.
 * @param time New GPS time format.
 * @param block Whether to wait for a confirmation from the receiver that the settings
 * have taken effect.
 * @return `true` if the settings were change successfully, `false` if an I/O problem occurred.
 */
bool CopernicusGPS::setFixMode(ReportType pos_fixmode, 
                               ReportType vel_fixmode,
                               AltMode alt,
                               PPSMode pps,
                               GPSTimeMode time,
                               bool block) {
    // request current IO settings
    beginCommand(CMD_IO_OPTIONS);
    endCommand();
    waitForPacket(RPT_IO_SETTINGS);
    
    uint8_t bytes[4];
    if (readDataBytes(bytes, 4) != 4) return false;
    if (not endReport()) return false;
    
    const uint8_t pos_mask = 0x13;
    const uint8_t vel_mask = 0x03;
    const uint8_t alt_mask = 0x04;
    const uint8_t pps_mask = 0x60;
    const uint8_t tme_mask = 0x01;
    
    // alter position fixmode
    switch (pos_fixmode) {
        case RPT_FIX_POS_LLA_32:
            bytes[0] = (bytes[0] & ~pos_mask) | 0x02; break;
        case RPT_FIX_POS_LLA_64:
            bytes[0] = (bytes[0] & ~pos_mask) | 0x12; break;
        case RPT_FIX_POS_XYZ_32:
            bytes[0] = (bytes[0] & ~pos_mask) | 0x01; break;
        case RPT_FIX_POS_XYZ_64:
            bytes[0] = (bytes[0] & ~pos_mask) | 0x11; break;
        default: break; // do nothing
    }
    // alter velocity fixmode
    switch (vel_fixmode) {
        case RPT_FIX_VEL_XYZ:
            bytes[1] = (bytes[1] & ~vel_mask) | 0x01; break;
        case RPT_FIX_VEL_ENU:
            bytes[1] = (bytes[1] & ~vel_mask) | 0x02; break;
        default: break; // do nothing
    }
    // alter other fixmode settings
    if (alt  != ALT_NOCHANGE) bytes[0] = (bytes[0] & ~alt_mask) | alt;
    if (pps  != PPS_NOCHANGE) bytes[2] = (bytes[2] & ~pps_mask) | pps;
    if (time != TME_NOCHANGE) bytes[2] = (bytes[2] & ~tme_mask) | time;
    
    beginCommand(CMD_IO_OPTIONS);
    writeDataBytes(bytes, 4);
    endCommand();
    
    if (block) {
        waitForPacket(RPT_IO_SETTINGS);
        flushToNextPacket();
    }
    
    return true;
}

/***********************
 * Report processing   *
 ***********************/

// will process this packet normally, unless it is of type `haltAt`, in which
// case the stream will be left with only the header consumed, ready for
// the caller to process. Pass RPT_NONE to always consume.
ReportType CopernicusGPS::implProcessOnePacket(bool block, ReportType haltAt) {
    // packets are of the form:
    //   <DLE> <rpt-id> <data bytes ...> <DLE> <ETX>
    //   literal <DLE> bytes embedded in data are sent as <DLE> <DLE>.
    while (true) {
        if (m_serial->available() <= 0) {
            if (block) blockForData();
            else return RPT_NONE;
        }
        int b = m_serial->read();
        if (b != CTRL_DLE) {
            // we're not at the start of a packet; find the end.
            if (not flushToNextPacket(block)) return RPT_NONE;
            continue;
        }
        if (m_serial->available() <= 0) blockForData();
        b = m_serial->read();
        if (b == CTRL_DLE) {
            // double-DLE; a literal, not a packet header. Find the end.
            if (not flushToNextPacket(block)) return RPT_NONE;
            continue;
        } else if (b == CTRL_ETX) {
            // we're at the apparent end of a packet. this should be
            // followed by the start of another. Go around the horn
            // and try again.
            continue;
        } else {
            ReportType rpt = static_cast<ReportType>(b);
            if (rpt == haltAt and haltAt != RPT_NONE) return rpt;
            else if (not processReport(rpt)) return RPT_ERROR;
            else return rpt;
        }
    } 
}

bool CopernicusGPS::processReport(ReportType type) {
    bool ok = true;
    switch (type) {
        case RPT_FIX_POS_LLA_32:
            ok = process_p_LLA_32(); break;
        case RPT_FIX_POS_LLA_64:
            ok = process_p_LLA_64(); break;
        case RPT_FIX_POS_XYZ_32:
            ok = process_p_XYZ_32(); break;
        case RPT_FIX_POS_XYZ_64:
            ok = process_p_XYZ_64(); break;
        case RPT_FIX_VEL_XYZ:
            ok = process_v_XYZ(); break;
        case RPT_FIX_VEL_ENU:
            ok = process_v_ENU(); break;
        case RPT_GPSTIME:
            ok = process_GPSTime(); break;
        case RPT_HEALTH:
            ok = process_health(); break;
        case RPT_ADDL_STATUS:
            ok = process_addl_status(); break;
        default:
            // give the user's packet processors a swipe
            PacketStatus st = PKT_IGNORE;
            for (int i = 0; i < m_n_listeners; i++) {
                st = m_listeners[i]->gpsPacket(type, this);
                if (st != PKT_IGNORE) {
                    ok = (st != PKT_ERROR);
                    break;
                }
            } 
            if (st != PKT_CONSUMED) {
                // consume the rest of this packet.
                flushToNextPacket(false);
            }
    }
    return ok;
}

bool CopernicusGPS::process_p_LLA_32() {
    m_pfix.type = RPT_FIX_POS_LLA_32;
    LLA_Fix<Float32> *fix = &m_pfix.lla_32;
    uint8_t buf[4];
    
    SAVE_BYTES(&fix->lat.bits, buf, 4);
    SAVE_BYTES(&fix->lng.bits, buf, 4);
    SAVE_BYTES(&fix->alt.bits, buf, 4);
    SAVE_BYTES(&fix->bias.bits, buf, 4);
    SAVE_BYTES(&fix->fixtime.bits, buf, 4);
    
    bool ok = endReport();
    if (not ok) m_vfix.type = RPT_ERROR;
    return ok;
}

bool CopernicusGPS::process_p_LLA_64() {
    m_pfix.type = RPT_FIX_POS_LLA_64;
    LLA_Fix<Float64> *fix = &m_pfix.lla_64;
    uint8_t buf[8];
    
    SAVE_BYTES(&fix->lat.bits, buf, 8);
    SAVE_BYTES(&fix->lng.bits, buf, 8);
    SAVE_BYTES(&fix->alt.bits, buf, 8);
    SAVE_BYTES(&fix->bias.bits, buf, 8);
    SAVE_BYTES(&fix->fixtime.bits, buf, 4);
    
    bool ok = endReport();
    if (not ok) m_vfix.type = RPT_ERROR;
    return ok;
}

bool CopernicusGPS::process_p_XYZ_32() {
    m_pfix.type = RPT_FIX_POS_XYZ_32;
    XYZ_Fix<Float32> *fix = &m_pfix.xyz_32;
    uint8_t buf[4];
    
    SAVE_BYTES(&fix->x.bits, buf, 4);
    SAVE_BYTES(&fix->y.bits, buf, 4);
    SAVE_BYTES(&fix->z.bits, buf, 4);
    SAVE_BYTES(&fix->bias.bits, buf, 4);
    SAVE_BYTES(&fix->fixtime.bits, buf, 4);
    
    bool ok = endReport();
    if (not ok) m_pfix.type = RPT_ERROR;
    return ok;
}

bool CopernicusGPS::process_p_XYZ_64() {
    m_pfix.type = RPT_FIX_POS_XYZ_64;
    XYZ_Fix<Float64> *fix = &m_pfix.xyz_64;
    uint8_t buf[8];
    
    SAVE_BYTES(&fix->x.bits, buf, 8);
    SAVE_BYTES(&fix->y.bits, buf, 8);
    SAVE_BYTES(&fix->z.bits, buf, 8);
    SAVE_BYTES(&fix->bias.bits, buf, 8);
    SAVE_BYTES(&fix->fixtime.bits, buf, 4);
    
    bool ok = endReport();
    if (not ok) m_pfix.type = RPT_ERROR;
    return ok;
}

bool CopernicusGPS::process_v_XYZ() {
    m_vfix.type = RPT_FIX_VEL_XYZ;
    XYZ_VFix *fix = &m_vfix.xyz;
    uint8_t buf[4];
    
    SAVE_BYTES(&fix->x.bits, buf, 4);
    SAVE_BYTES(&fix->y.bits, buf, 4);
    SAVE_BYTES(&fix->z.bits, buf, 4);
    SAVE_BYTES(&fix->bias.bits, buf, 4);
    SAVE_BYTES(&fix->fixtime.bits, buf, 4);
    
    bool ok = endReport();
    if (not ok) m_vfix.type = RPT_ERROR;
    return ok;
}

bool CopernicusGPS::process_v_ENU() {
    m_vfix.type = RPT_FIX_VEL_ENU;
    ENU_VFix *fix = &m_vfix.enu;
    uint8_t buf[4];
    
    SAVE_BYTES(&fix->e.bits, buf, 4);
    SAVE_BYTES(&fix->n.bits, buf, 4);
    SAVE_BYTES(&fix->u.bits, buf, 4);
    SAVE_BYTES(&fix->bias.bits, buf, 4);
    SAVE_BYTES(&fix->fixtime.bits, buf, 4);
    
    bool ok = endReport();
    if (not ok) m_vfix.type = RPT_ERROR;
    return ok;
}

bool CopernicusGPS::process_GPSTime() {
    uint8_t buf[4];
    
    SAVE_BYTES(&m_time.time_of_week.bits, buf, 4);
    SAVE_BYTES(&m_time.week_no, buf, 2);
    SAVE_BYTES(&m_time.utc_offs, buf, 4);
    
    bool ok = endReport();
    if (not ok)  m_time.time_of_week.bits = 0xBF800000; // -1
    return ok;
}

bool CopernicusGPS::process_health() {
    uint8_t buf[2];
    if (readDataBytes(buf, 2) != 2) return false;
    m_status.health = static_cast<GPSHealth>(buf[0]);
    
    bool ok = endReport();
    if (not ok) m_status.health = HLTH_UNKNOWN;
    return ok;
}

bool CopernicusGPS::process_addl_status() {
    uint8_t buf[3];
    if (readDataBytes(buf, 3) != 3) return false;
    m_status.rtclock_unavailable = (buf[1] & 0x02) != 0;
    m_status.almanac_incomplete  = (buf[1] & 0x08) != 0;
    
    return endReport();
}

bool CopernicusGPS::process_sbas_status() {
    uint8_t buf;
    if (readDataBytes(&buf, 1) != 1) return false;
    m_status.sbas_corrected = (buf & 0x01) != 0;
    m_status.sbas_enabled   = (buf & 0x02) != 0;
    
    return endReport();
}

/***************************
 * access                  *
 ***************************/

/**
 * Get the monitored Serial IO object.
 */
HardwareSerial* CopernicusGPS::getSerial() {
    return m_serial;
}

/**
 * Get the status and health of the reciever.
 * If the unit has a GPS lock, `getStatus().health` will equal `HLTH_DOING_FIXES`.
 */
const GPSStatus& CopernicusGPS::getStatus() const {
    return m_status;
}

/**
 * Get the most current position fix.
 */
const PosFix& CopernicusGPS::getPositionFix() const {
    return m_pfix;
}

/**
 * Get the most current velocity fix.
 */
const VelFix& CopernicusGPS::getVelocityFix() const {
    return m_vfix;
}

/**
 * Get the most recent GPS time report. For accurate current time,
 * this datum must be correlated with a PPS pulse signal.
 */
const GPSTime& CopernicusGPS::getGPSTime() const {
    return m_time;
}

/**
 * Add a `GPSPacketProcessor` to be notified of incoming TSIP packets. At most
 * `MAX_PKT_PROCESSORS` (8) are supported at a time. 
 * @param pcs Processor to add.
 * @return `false` if there was not enough space to add the processor, `true` otherwise.
 */
bool CopernicusGPS::addPacketProcessor(GPSPacketProcessor *pcs) {
    for (int i = 0; i < m_n_listeners; i++) {
        if (m_listeners[i] == pcs) return true;
    }
    if (m_n_listeners >= MAX_PKT_PROCESSORS) return false;
    m_listeners[m_n_listeners++] = pcs;
    return true;
}

/**
 * Cease to notify the given `GPSPacketProcessor` of incoming TSIP packets.
 * @param pcs Processor to remove.
 */
void CopernicusGPS::removePacketProcessor(GPSPacketProcessor *pcs) {
    bool found = false;
    for (int i = 0; i < m_n_listeners; i++) {
        if (m_listeners[i] == pcs) {
            found = true;
        }
        if (found and i < m_n_listeners - 1) {
            m_listeners[i] = m_listeners[i + 1];
        }
    }
    if (found) {
        m_n_listeners = m_n_listeners - 1;
    }
}

/****************************
 * gps listener             *
 ****************************/

GPSPacketProcessor::~GPSPacketProcessor() {}