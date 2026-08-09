#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <sensor_msgs/msg/nav_sat_fix.h>

#define USING_GPS

extern sensor_msgs__msg__NavSatFix gps_msg;
extern SFE_UBLOX_GNSS GNSS;

void obc_setup_gps();
void updatePVTData(UBX_NAV_PVT_data_t* ubx_nav);
void update_gps();

#endif