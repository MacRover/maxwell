#include "gps.h"

sensor_msgs__msg__NavSatFix gps_msg;
SFE_UBLOX_GNSS GNSS;

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);
struct timespec tp;

void updatePVTData(UBX_NAV_PVT_data_t* ubx_nav)
{
    clock_gettime(CLOCK_REALTIME, &tp);
    gps_msg.header.stamp.sec = tp.tv_sec;
    gps_msg.header.stamp.nanosec = tp.tv_nsec;
    gps_msg.header.frame_id.data = "gps_frame";
    gps_msg.header.frame_id.size = 9;
    gps_msg.header.frame_id.capacity = 9;

    gps_msg.latitude = (double)( ubx_nav->lat ) * 0.0000001;
    gps_msg.longitude = (double)( ubx_nav->lon ) * 0.0000001;
    gps_msg.altitude = (double)( ubx_nav->hMSL ) * 0.001;

    double H_m = ubx_nav->hAcc * 0.001;
    double V_m = ubx_nav->vAcc * 0.001;
    gps_msg.position_covariance[0] = H_m*H_m;
    gps_msg.position_covariance[4] = H_m*H_m;
    gps_msg.position_covariance[8] = V_m*V_m;
    gps_msg.position_covariance_type = 
        sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_DIAGONAL_KNOWN;

    if (ubx_nav->fixType < 2)
        gps_msg.status.status = 
            sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
    else
        gps_msg.status.status = 
            sensor_msgs__msg__NavSatStatus__STATUS_FIX;

    gps_msg.status.service = 
        sensor_msgs__msg__NavSatStatus__SERVICE_GPS;
}

void obc_setup_gps()
{
#ifdef USING_GPS
    while (!GNSS.begin(Serial6)) { delay(100); }
    GNSS.setUART1Output(COM_TYPE_UBX);
    GNSS.setMeasurementRate(33.333);
    GNSS.setNavigationRate(6);
    GNSS.saveConfiguration();
    GNSS.setAutoPVTcallbackPtr(&updatePVTData);
#endif
}

void update_gps()
{
#ifdef USING_GPS
    GNSS.checkUblox();
    GNSS.checkCallbacks();
#endif
}