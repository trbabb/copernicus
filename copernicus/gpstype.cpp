#include <cstddef>
#include "gpstype.h"

#ifndef NULL
#define NULL (0)
#endif

/***************************
 * Fix types               *
 ***************************/

PosFix::PosFix() : type(RPT_NONE) {}
VelFix::VelFix() : type(RPT_NONE) {}

const LLA_Fix<Float32>* PosFix::getLLA_32() const {
    if (type == RPT_FIX_POS_LLA_32) return &lla_32;
    else return NULL;
}

const LLA_Fix<Float64>* PosFix::getLLA_64() const {
    if (type == RPT_FIX_POS_LLA_64) return &lla_64;
    else return NULL;
}

const XYZ_Fix<Float32>* PosFix::getXYZ_32() const {
    if (type == RPT_FIX_POS_XYZ_32) return &xyz_32;
    else return NULL;
}

const XYZ_Fix<Float64>* PosFix::getXYZ_64() const {
    if (type == RPT_FIX_POS_XYZ_64) return &xyz_64;
    else return NULL;
}

const XYZ_VFix *VelFix::getXYZ() const {
    if (type == RPT_FIX_VEL_XYZ) return &xyz;
    else return NULL;
}

const ENU_VFix *VelFix::getENU() const {
    if (type == RPT_FIX_VEL_ENU) return &enu;
    else return NULL;
}

/***************************
 * GPSStatus               *
 ***************************/

GPSStatus::GPSStatus():
        health(HLTH_UNKNOWN),
        n_satellites(0),
        almanac_incomplete(true),
        rtclock_unavailable(true),
        sbas_enabled(false),
        sbas_corrected(false) {}