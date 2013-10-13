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
 * @param serial Arduino serial stream to monitor. 0 is `Serial`, 1 is 
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
}

/***************************
 * i/o                     *
 ***************************/

/**
 * Being a TSIP command by sending the header bytes for the given
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
 * @param bytes Data bytes to write.
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
            processReport(rpt);
            return rpt;
        }
    }
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
 * Report processing   *
 ***********************/


bool CopernicusGPS::processReport(ReportType type) {
    bool ok = false;
    bool unknown = false;
    switch (type) {
        case RPT_FIX_POS_LLA_SP:
            ok = process_p_LLA_32(); break;
        case RPT_FIX_POS_LLA_DP:
            ok = process_p_LLA_64(); break;
        case RPT_FIX_POS_XYZ_SP:
            ok = process_p_XYZ_32(); break;
        case RPT_FIX_POS_XYZ_DP:
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
            // consume the rest of this packet, and
            // process the next one if it's available.
            unknown = true;
            flushToNextPacket(false);
            break;
    }
    return unknown or ok;
}

bool CopernicusGPS::process_p_LLA_32() {
    m_pfix.type = RPT_FIX_POS_LLA_SP;
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
    m_pfix.type = RPT_FIX_POS_LLA_DP;
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
    m_pfix.type = RPT_FIX_POS_XYZ_SP;
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
    m_pfix.type = RPT_FIX_POS_XYZ_DP;
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
 * Add a listener to be notified of GPS events. At most
 * `N_GPS_LISTENERS` (16) are supported at a time. 
 * @param lsnr Listener to add.
 * @return `false` if there was not enough space to add the listener, `true` otherwise.
 */
bool CopernicusGPS::addListener(GPSPacketProcessor *lsnr) {
    for (int i = 0; i < m_n_listeners; i++) {
        if (m_listeners[i] == lsnr) return true;
    }
    if (m_n_listeners >= N_GPS_LISTENERS) return false;
    m_listeners[m_n_listeners++] = lsnr;
    return true;
}

/**
 * Cease to notify the given `GPSListener` of GPS events.
 * @param lsnr Listener to remove.
 */
void CopernicusGPS::removeListener(GPSPacketProcessor *lsnr) {
    bool found = false;
    for (int i = 0; i < m_n_listeners; i++) {
        if (m_listeners[i] == lsnr) {
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