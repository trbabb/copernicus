/* 
 * File:   chunk.h
 * Author: tbabb
 * 
 * Code for copying bytes from a big-endian stream.
 *
 * Created on October 7, 2013, 1:44 AM
 */

#ifndef CHUNK_H
#define	CHUNK_H

#include "gpstype.h"

////////// Unsigned ints //////////

inline void copy_network_order(uint16_t *i, uint8_t bytes[2]) {
    *i = ((uint16_t)(bytes[0]) << 8) | bytes[1];
}
inline void copy_network_order(uint32_t *i, uint8_t bytes[4]) {
    *i  = (uint32_t)(bytes[0]) << 24;
    *i |= (uint32_t)(bytes[1]) << 16;
    *i |= (uint32_t)(bytes[2]) <<  8;
    *i |= (uint32_t)(bytes[3]);
}

inline void copy_network_order(uint64_t *i, uint8_t bytes[8]) {
    *i  = (uint64_t)(bytes[0]) << 56;
    *i |= (uint64_t)(bytes[1]) << 48;
    *i |= (uint64_t)(bytes[2]) << 40;
    *i |= (uint64_t)(bytes[3]) << 32;
    *i |= (uint64_t)(bytes[4]) << 24;
    *i |= (uint64_t)(bytes[5]) << 16;
    *i |= (uint64_t)(bytes[6]) <<  8;
    *i |= (uint64_t)(bytes[7]);
}

////////// Signed ints //////////

inline void copy_network_order(int16_t *i, uint8_t bytes[2]) {
    copy_network_order(reinterpret_cast<uint16_t*>(i), bytes);
}

inline void copy_network_order(int32_t *i, uint8_t bytes[4]) {
    copy_network_order(reinterpret_cast<uint32_t*>(i), bytes);
}

inline void copy_network_order(int64_t *i, uint8_t bytes[8]) {
    copy_network_order(reinterpret_cast<uint64_t*>(i), bytes);
}

////////// Floats //////////

inline void copy_network_order(Float32 *f, uint8_t bytes[4]) {
    copy_network_order(&f->bits, bytes);
}

inline void copy_network_order(Float64 *f, uint8_t bytes[8]) {
    copy_network_order(&f->bits, bytes);
}

#endif	/* CHUNK_H */

