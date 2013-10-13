/* 
 * File:   gpstype.h
 * Author: tbabb
 *
 * Created on October 7, 2013, 11:04 PM
 */

#ifndef GPSTYPE_H
#define	GPSTYPE_H

#include <stdint.h>
#include <cfloat>

/***************************
 * enums                   *
 ***************************/

enum CommandID {
    
};

enum ReportType {
    /// Set if a known packet was corrupted or could not be processed.
    RPT_ERROR = -1,
    /// Set in fixes if the fix is invalid and/or no fix has been obtained yet.
    RPT_UNKNOWN = 0x00,
    
    // position/velocity fixes
    
    /// Position fix, Lat/Lng/Alt, single-precision (32 bit).
    RPT_FIX_POS_LLA_SP = 0x4A,
    /// Position fix, Lat/Lng/Alt, double-precision (64 bit).
    RPT_FIX_POS_LLA_DP = 0x84,
    /// Position fix, XYZ Earth-centered Earth-fixed, single-precision.
    RPT_FIX_POS_XYZ_SP = 0x42,
    /// Position fix, XYZ Earth-centered Earth-fixed, double-precision.
    RPT_FIX_POS_XYZ_DP = 0x83,
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
    RPT_SBAS_MODE   = 0x82
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
    HLTH_SATELLITES_OVERDETERMINED = 0xBB
};


/***************************
 * storage types           *
 ***************************/

// many/most arduino chipsets do not support 32 bit floats,
// but we still need to be able to store the bits of a 32-bit float somehow.

union Float32 {
    uint32_t bits;
#if FLT_MANT_DIG == 24
    float f;
#elif DBL_MANT_DIG == 24
    double f;
#endif
};

union Float64 {
    uint64_t bits;
#if DBL_MANT_DIG == 53
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

struct PosFix {
    
    ReportType type;
    
    union {
        XYZ_Fix<Float32> xyz_32;
        XYZ_Fix<Float64> xyz_64;
        LLA_Fix<Float32> lla_32;
        LLA_Fix<Float64> lla_64;
    };
    
    PosFix();
    
    LLA_Fix<Float32> *getLLA_32();
    LLA_Fix<Float64> *getLLA_64();
    XYZ_Fix<Float32> *getXYZ_32();
    XYZ_Fix<Float64> *getXYZ_64();
};

struct VelFix {
    ReportType type;
    
    union {
        XYZ_VFix xyz;
        ENU_VFix enu;
    };
    
    VelFix();
    
    XYZ_VFix *getXYZ();
    ENU_VFix *getENU();
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

#endif	/* GPSTYPE_H */

