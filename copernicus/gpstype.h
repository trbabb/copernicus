/* 
 * File:   gpstype.h
 * Author: tbabb
 *
 * Created on October 7, 2013, 11:04 PM
 */

#ifndef GPSTYPE_H
#define	GPSTYPE_H

#include <stdint.h>
#include <float.h>

class CopernicusGPS; // fwd decl

/**
 * @defgroup datapoint
 * @brief Various GPS datapoint types.
 */

/**
 * @addtogroup datapoint
 * @{
 */

/***************************
 * enums                   *
 ***************************/

enum CommandID {
    CMD_IO_OPTIONS = 0x35
};

enum ReportType {
    /// Set if a known packet was corrupted or could not be processed.
    RPT_ERROR = -1,
    /// Set in fixes if the fix is invalid and/or no fix has been obtained yet.
    RPT_NONE = 0x00,
    
    // position/velocity fixes
    
    /// Position fix, Lat/Lng/Alt, single-precision (32 bit).
    RPT_FIX_POS_LLA_32 = 0x4A,
    /// Position fix, Lat/Lng/Alt, double-precision (64 bit).
    RPT_FIX_POS_LLA_64 = 0x84,
    /// Position fix, XYZ Earth-centered Earth-fixed, single-precision.
    RPT_FIX_POS_XYZ_32 = 0x42,
    /// Position fix, XYZ Earth-centered Earth-fixed, double-precision.
    RPT_FIX_POS_XYZ_64 = 0x83,
    /// Velocity fix, XYZ Earth-centered Earth-fixed.
    RPT_FIX_VEL_XYZ    = 0x43,
    /// Velocity fix, East/North/Up.
    RPT_FIX_VEL_ENU    = 0x56,
    
    // other auto-reports
    
    /// GPS time report.
    RPT_GPSTIME     = 0x41,
    /// Receiver health report.
    RPT_HEALTH      = 0x46,
    /// Additional receiver status report (almanac / realtime clock availability).
    RPT_ADDL_STATUS = 0x4B,
    /// Satellite report.
    RPT_SATELLITES  = 0x6d,
    /// SBAS (Satellite-based augmentation system) mode report.
    RPT_SBAS_MODE   = 0x82,
    
    // replies
    
    /// GPS IO settings.
    RPT_IO_SETTINGS = 0x55,
};

enum GPSHealth {
    /// Set if GPS health has not been established yet.
    HLTH_UNKNOWN                   = 0xFF,
    /// Set if reciever has a GPS lock and is obtaining valid fixes.
    HLTH_DOING_FIXES               = 0x00,
    /// Set if GPS time has not been obtained yet.
    HLTH_NO_GPSTIME                = 0x01,
    /// Set if satellite geometry is too poor to obtain a fix.
    HLTH_PDOP_TOO_HIGH             = 0x03,
    /// Set if the chosen SV is unavailable.
    HLTH_SV_UNAVAILABLE            = 0x04, 
    /// Set if no useable satellites have been locked.
    HLTH_SATELLITES_NONE           = 0x08,
    /// Set if only one useable satellite has been locked.
    HLTH_SATELLITES_ONE            = 0x09,
    /// Set if only two useable satellites have been locked.
    HLTH_SATELLITES_TWO            = 0x0A,
    /// Set if only three useable satellites have been locked.
    HLTH_SATELLITES_THREE          = 0x0B,
    /// Set if operating in overdetermined mode.
    HLTH_SATELLITES_OVERDETERMINED = 0xBB,
};

enum PacketStatus {
    /// Indicates the GPSPacketProcessor does not wish to intercept this packet and no bytes have been consumed.
    PKT_IGNORE,
    /// Indicates that the GPSPacketProcessor has consumed and processed the packet, including the end-of-packet sequence.
    PKT_CONSUMED,
    /// Indicates that an error has occurred while processing the packet, and the stream should be advanced to a safe state.
    PKT_ERROR,
    /// Indicates that the GPSPacketProcessor has consumed some bytes of the packet, and that the remainder of the packet should be consumed.
    PKT_PARTIAL,
};

enum AltMode {
    /// Height above WGS-84 ellipsoid.
    ALT_HAE  = 0x00,
    /// Height above mean sea level.
    ALT_MSL  = 0x01,
    /// Flag to leave altitude mode unchanged.
    ALT_NOCHANGE = 0xFF,
};

enum PPSMode {
    /// PPS always on.
    PPS_ALWAYS = 0x00,
    /// PPS fix-based.
    PPS_FIX    = 0x20,
    /// PPS off.
    PPS_OFF    = 0x40,
    /// Flag to leave PPS unchanged.
    PPS_NOCHANGE   = 0x60,
};

enum GPSTimeMode {
    /// Report GPS time.
    TME_GPSTIME  = 0x00,
    /// Report UTC.
    TME_UTCTIME  = 0x01,
    /// Flag to leave time reporting mode unchanged.
    TME_NOCHANGE = 0xFF
};

/***************************
 * storage types           *
 ***************************/

// many/most arduino chipsets do not support 64 bit floats,
// but we still need to be able to store the bits of a 64-bit float somehow.

union Float32 {
    /// The bits of an IEEE 754 32-bit float.
    uint32_t bits;
#if FLT_MANT_DIG == 24 || defined(PARSING_DOXYGEN)
    /**
     * @brief The actual `float` value of the datapoint.
     * 
     * Unavailable if `float` is not a 32-bit IEEE 754.
     */
    float f;
#elif DBL_MANT_DIG == 24
    double f;
#endif
};

union Float64 {
    /// The bits of an IEEE 754 64-bit float.
    uint64_t bits;
#if DBL_MANT_DIG == 53 || defined(PARSING_DOXYGEN)
    /**
     * @brief The actual `double` value of the datapoint.
     * 
     * Unavailable if `double` is not a 64-bit IEEE 754. Only some Arduino
     * boards support this, such as the Arduino Due.
     */
    double d;
#elif LDBL_MANT_DIG == 53
    long double d;
#endif
};

/***************************
 * datapoints              *
 ***************************/

// all angles are in radians.
// fix times are -1 if the fix is not valid.

template <typename T>
struct LLA_Fix {
    T lat;
    T lng;
    T alt;
    T bias;
    Float32 fixtime;
};

template <typename T>
struct XYZ_Fix {
    T x;
    T y;
    T z;
    T bias;
    Float32 fixtime;
};

struct XYZ_VFix {
    Float32 x;
    Float32 y;
    Float32 z;
    Float32 bias;
    Float32 fixtime;
};

struct ENU_VFix {
    Float32 e;
    Float32 n;
    Float32 u;
    Float32 bias;
    Float32 fixtime;
};

/**
 * @brief Position fix.
 * 
 * Depending on the reporting mode of the receiver, this structure may store
 * a report of type `RPT_FIX_POS_LLA_32`, `RPT_FIX_POS_LLA_64`, `RPT_FIX_POS_XYZ_32`,
 * `RPT_FIX_POS_XYZ_64`, or `RPT_NONE`. The stored type is indicated by 
 * `PosFix.type`; use one of the four access methods to obtain an object storing 
 * the actual position data. Access methods which don't currently correspond to 
 * the store type will return `NULL`.
 * 
 * If the report type is unknown, there is not yet a valid fix and all accessors 
 * will return `NULL`.
 * 
 * For example:
 *     
 *      const PosFix &fix = gps.GetPositionFix();
 *      if (fix.type == RPT_FIX_POS_LLA_32) {
 *          LLA_Fix<Float32> fixdata = fix.getLLA_32();
 *          // ...
 *      } else if (fix.type == RPT_FIX_POS_XYZ_32) {
 *          XYZ_FIX<Float32> fixdata = fix.getXYZ_32();
 *          // ...
 *      } // etc.
 *     
 */
struct PosFix {
    
    /// Format of stored position fix.
    ReportType type;
    
    PosFix();
    
    const LLA_Fix<Float32> *getLLA_32() const;
    const LLA_Fix<Float64> *getLLA_64() const;
    const XYZ_Fix<Float32> *getXYZ_32() const;
    const XYZ_Fix<Float64> *getXYZ_64() const;
    
protected:
    
    union {
        XYZ_Fix<Float32> xyz_32;
        XYZ_Fix<Float64> xyz_64;
        LLA_Fix<Float32> lla_32;
        LLA_Fix<Float64> lla_64;
    };
    
    friend class CopernicusGPS;
};

/**
 * @brief Velocity fix.
 * 
 * Depending on the reporting mode of the receiver, this structure may store
 * a report of type `RPT_FIX_VEL_XYZ`, `RPT_FIX_VEL_ENU`, or `RPT_NONE`. The 
 * stored type is indicated by `VelFix.type`; use one of the two access methods 
 * to obtain an object storing the actual velocity data. Access methods which 
 * don't currently correspond to the store type will return `NULL`. 
 * 
 * If the report type is unknown, there is not yet a valid fix and all accessors 
 * will return `NULL`.
 * 
 * For example:
 *     
 *     const VelFix &fix = gps.GetVelocityFix();
 *     if (fix.type == RPT_FIX_VEL_ENU) {
 *         ENU_VFix fixdata = fix.getENU();
 *         // ...
 *     } else {
 *         XYZ_VFix fixdata = fix.getXYZ();
 *         // ...
 *     }
 * .
 */
struct VelFix {
    /// Format of stored velocity fix.
    ReportType type;
    
    VelFix();
    
    const XYZ_VFix *getXYZ() const;
    const ENU_VFix *getENU() const;
    
protected:
    
    union {
        XYZ_VFix xyz;
        ENU_VFix enu;
    };
    
    friend class CopernicusGPS;
};

struct GPSTime {
    Float32 time_of_week;
    int16_t week_no;
    Float32 utc_offs;
};

struct GPSStatus {
    GPSStatus();
    
    GPSHealth health;         // pkt 0x46
    int n_satellites;         // pkt 0x6d
    bool almanac_incomplete;  // pkt 0x4b
    bool rtclock_unavailable; // pkt 0x4b
    bool sbas_enabled;        // pkt 0x82
    bool sbas_corrected;      // pkt 0x82
};

/// @} // addtogroup datapoint

#endif	/* GPSTYPE_H */

