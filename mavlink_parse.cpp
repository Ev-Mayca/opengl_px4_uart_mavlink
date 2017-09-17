/*************************************************************************
	> File Name: mavlink_parse.cpp
	> Author: Ev
	> Mail: wang2011yiwei@sina.com 
	> Created Time: 2017年03月25日 星期六 15时21分46秒
 ************************************************************************/

//------------------------ Include Files -------------------------------//

#include "mavlink_parse.hpp"


//--------------------------- Veriable ---------------------------------//
using namespace std;
//--------------------- Function Prototype -----------------------------//

//------------------------- Function -----------------------------------//
MavlinkParse::MavlinkParse()
{
	// /dev/ttyHSU1 USB1
	serial = new PortPoll("/dev/ttyUSB0",230400,0,8,1,'N');
	// serial = new PortPoll("/dev/ttyUSB0",57600,0,8,1,'N');
	// serial = new PortPoll("/dev/ttyHSU0",230400,0,8,1,'N');
	// std::cout << "open mavlink " << std::endl;

}

MavlinkParse::~MavlinkParse()
{
	// std::cout << "close mavlink " << std::endl;
	delete serial;
}

int MavlinkParse::get_message(mavlink_message_t &message)
{
	char serial_c = 0;
	unsigned int count = 0;

	mavlink_status_t status;
	unsigned int decodeState = 0;
	while(1){
		serial->serial_read(&serial_c,1);
		// if(serial_c == 0xfe)printf("get ");
		decodeState = mavlink_parse_char(0, (uint8_t)(serial_c), &message, &status);
		if(decodeState == 1){
			// if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {


            //     mavlink_heartbeat_t heartbeat;
            //     mavlink_msg_heartbeat_decode(&message, &heartbeat);
            //     emit vehicleHeartbeatInfo(link, message.sysid, message.compid, heartbeat.mavlink_version, heartbeat.autopilot, heartbeat.type);
            // }
			// cout << message.msgid << endl;
			// printf("id:%d\n",message.msgid);
			// memcpy(&_message,&message,sizeof(message));
			
			return 1;
		}	
		if((count ++) == 1024)break;
	}
	// serial->serial_read(&serial_c,1);
	// decodeState = mavlink_parse_char(0, (uint8_t)(serial_c), &message, &status);
	// static char serial_c = 0;
	// char *p = buf;
	// do{
	// 	*p ++ = serial_c;
	// 	serial->serial_read(&serial_c,1);

	// }while(serial_c != 0xfe);

	// return (*buf == 0xfe)?0:-1;
	return 0;
}

int MavlinkParse::handlemavlinkScaledImu(mavlink_message_t &message,mavlink_highres_imu_t &scaled_imu)
{
	mavlink_msg_highres_imu_decode(&message,&scaled_imu);
	return 0;
}

int MavlinkParse::handlemavlinkGpsStatus(mavlink_message_t &message,mavlink_gps_status_t &gps_status)
{
	mavlink_msg_gps_status_decode(&message,&gps_status);
	return 0;
}
int MavlinkParse::handlemavlinkGpsRawInit(mavlink_message_t &message,mavlink_gps_raw_int_t &gps_raw_init)
{
    
    mavlink_msg_gps_raw_int_decode(&message, &gps_raw_init);
	return 0;
}

int MavlinkParse::handlemavlinkAttitudeQuaternion(mavlink_message_t &message,mavlink_attitude_quaternion_t &attitude_quaternion)
{
    
    mavlink_msg_attitude_quaternion_decode(&message, &attitude_quaternion);
	return 0;
}

int MavlinkParse::handlemavlinkLocalPosition(mavlink_message_t &message,mavlink_local_position_ned_t &pos)
{
	mavlink_msg_local_position_ned_decode(&message,&pos);
	return 0;
}