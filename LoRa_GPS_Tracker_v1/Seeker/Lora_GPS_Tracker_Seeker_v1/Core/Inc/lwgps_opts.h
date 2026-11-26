/**
 * lwgps_opts.h  ->  put this in Core/Inc (must be on the include path)
 *
 * Statements are enabled explicitly so we don't depend on the library's
 * default-settings header being picked up.
 */
#ifndef LWGPS_HDR_OPTS_H
#define LWGPS_HDR_OPTS_H

/* Cortex-M3 has no FPU: float is far cheaper than double */
#define LWGPS_CFG_DOUBLE            0

/* NMEA statements we want parsed */
#define LWGPS_CFG_STATEMENT_GPGGA   1   /* lat, lon, altitude, sats in use, time */
#define LWGPS_CFG_STATEMENT_GPGSA   1   /* DOP, fix mode */
#define LWGPS_CFG_STATEMENT_GPGSV   1   /* sats in view */
#define LWGPS_CFG_STATEMENT_GPRMC   1   /* is_valid, speed, course, date */

#define LWGPS_CFG_STATEMENT_GPGSV_SAT_DET   0   /* per-satellite details, not needed */
#define LWGPS_CFG_STATEMENT_PUBX_TIME       0
#define LWGPS_CFG_CRC                       1   /* reject corrupted sentences */
#define LWGPS_CFG_SATS_IN_USE_SIZE          24

#endif /* LWGPS_HDR_OPTS_H */
