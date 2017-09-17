/*************************************************************************
	> File Name: mavlink_parse.h
	> Author: Ev
	> Mail: wang2011yiwei@sina.com 
	> Created Time: 2017年03月13日 星期一 21时46分34秒
 ************************************************************************/
#ifndef __MAVLINK_PARSEL_H_
#define __MAVLINK_PARSEL_H_

//--------------------------- Define -----------------------------------//
#include <iostream>
#include "serial.hpp"
#include "mavlink.h"


#define MAVLINK_BUF_MAX 1024

class MavlinkParse
{
	public:
		MavlinkParse();
		~MavlinkParse();

		int get_message(mavlink_message_t &message);
		int handlemavlinkScaledImu(mavlink_message_t &message,mavlink_highres_imu_t &scaled_imu);
		int handlemavlinkGpsStatus(mavlink_message_t &message,mavlink_gps_status_t &gps_status);
		int handlemavlinkGpsRawInit(mavlink_message_t &message,mavlink_gps_raw_int_t &gps_raw_init);
		int handlemavlinkAttitudeQuaternion(mavlink_message_t &message,mavlink_attitude_quaternion_t &attitude_quaternion);
		int handlemavlinkLocalPosition(mavlink_message_t &message,mavlink_local_position_ned_t &pos);
		PortPoll *serial;
	private:
		
		

};
#endif	//__MAVLINK_PARSEL_H_
