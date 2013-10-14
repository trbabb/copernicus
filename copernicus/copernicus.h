/* 
 * File:   copernicus.h
 * Author: tbabb
 *
 * Created on October 5, 2013, 10:35 PM
 * 
 * Note: It would be entirely possible, and not difficult, to extract
 * this module's dependency on the Arduino by replacing m_serial with
 * any reasonable buffered serial IO class.
 */

//TODO: support GPS time
//TODO: add commands
//TODO: fixp support

#ifndef COPERNICUS_H
#define	COPERNICUS_H

#define CTRL_DLE 0x10
#define CTRL_ETX 0x03

/**
 * @defgroup monitor
 * @brief Main classes for monitoring and commanding the Trimble Copernicus.
 */

/**
 * @addtogroup monitor
 * @{
 */

// arduino doesn't support std::vector
// so today we will be violating the zero/one/infinity rule.
#define MAX_PKT_PROCESSORS 8

#define TSIP_BAUD_RATE 38400

#include "gpstype.h"
#include "Arduino.h"

class CopernicusGPS; // fwd decl

/***************************
 * Listener class          *
 ***************************/

/**
 * @brief Class for directly intercepting and processing TSIP packets.
 * 
 * This provides a mechanism by which a client may make use of Trimble packets
 * which are not directly monitored/implemented by this API.
 * 
 * Only packets not monitored by the CopernicusGPS class will be passed on to 
 * registered GPSPacketProcessors.
 */
class GPSPacketProcessor {
public:
    virtual ~GPSPacketProcessor();
    
    /**
     * Called when a new TSIP packet has arrived. The packet header (`DLE` byte
     * and report ID) will have already been consumed. 
     * 
     * This function must not leave the stream in the middle of a `DLE` escape
     * sequence; that is to say an even number of `DLE` bytes must be consumed. 
     * 
     * @param type Type of TSIP report waiting in the serial stream.
     * @param gps GPS module which intercepted the report.
     * @return A `PacketStatus` indicating the state of the stream.
     */
    virtual PacketStatus gpsPacket(ReportType type, CopernicusGPS *gps) = 0;
};

/***************************
 * copernicus class        *
 ***************************/

// - what does the fix time mean / how does it relate to GPS time?
//   - the fix time is the time at which the fix was acquired. it will generally
//     be a few seconds in the past. Use your sync'd current GPS time 
//     to figure out how that relates to "now".
// - how does the GPS time relate to the last/next PPS?
//   - reported GPS time is that of the last PPS. So at the next
//     PPS pulse, add 1 to the captured GPS time, and that's the current time.

/**
 * @brief Class for communication with Trimble Copernicus GPS chip.
 */
class CopernicusGPS {
public:
    CopernicusGPS(int serial=0);
    
    ReportType processOnePacket(bool block=false);
    void waitForPacket(ReportType type);
    
    void beginCommand(CommandID cmd);
    void writeDataBytes(const uint8_t *bytes, int n);
    int  readDataBytes(uint8_t *dst, int n);
    void endCommand();
    
    bool setFixMode(ReportType pos_fixmode,
                    ReportType vel_fixmode,
                    AltMode alt=ALT_NOCHANGE,
                    PPSMode pps=PPS_NOCHANGE,
                    GPSTimeMode time=TME_NOCHANGE,
                    bool block=false);
    
    HardwareSerial  *getSerial();
    const PosFix&    getPositionFix() const;
    const VelFix&    getVelocityFix() const;
    const GPSTime&   getGPSTime() const;
    const GPSStatus& getStatus() const;
    
    bool addPacketProcessor(GPSPacketProcessor *pcs);
    void removePacketProcessor(GPSPacketProcessor *pcs);
    
private:
    
    ReportType implProcessOnePacket(bool block, ReportType haltAt);
    
    bool processReport(ReportType type);
    
    bool process_p_LLA_32();
    bool process_p_LLA_64();
    bool process_p_XYZ_32();
    bool process_p_XYZ_64();
    bool process_v_XYZ();
    bool process_v_ENU();
    bool process_GPSTime();
    bool process_health();
    bool process_addl_status();
    bool process_sbas_status();
    
    // todo: fix this busy wait.
    inline void blockForData() { while (m_serial->available() <= 0) {} }
    bool flushToNextPacket(bool block=true);
    bool endReport();
    
    HardwareSerial *m_serial;
    PosFix    m_pfix;
    VelFix    m_vfix;
    GPSTime   m_time;
    GPSStatus m_status;
    GPSPacketProcessor *m_listeners[MAX_PKT_PROCESSORS];
    uint8_t m_n_listeners;
};

/// @} // addtogroup monitor

#endif	/* COPERNICUS_H */

