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

LLA_Fix<Float32>* PosFix::getLLA_32() {
    if (type == RPT_FIX_POS_LLA_SP) return &lla_32;
    else return NULL;
}

LLA_Fix<Float64>* PosFix::getLLA_64() {
    if (type == RPT_FIX_POS_LLA_DP) return &lla_64;
    else return NULL;
}

XYZ_Fix<Float32>* PosFix::getXYZ_32() {
    if (type == RPT_FIX_POS_XYZ_SP) return &xyz_32;
    else return NULL;
}

XYZ_Fix<Float64>* PosFix::getXYZ_64() {
    if (type == RPT_FIX_POS_XYZ_DP) return &xyz_64;
    else return NULL;
}

XYZ_VFix *VelFix::getXYZ() {
    if (type == RPT_FIX_VEL_XYZ) return &xyz;
    else return NULL;
}

ENU_VFix *VelFix::getENU() {
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