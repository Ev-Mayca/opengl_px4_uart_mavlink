/*************************************************************************
	> File Name: serial.h
	> Author: Ev
	> Mail: wang2011yiwei@sina.com 
	> Created Time: 2017年03月13日 星期一 21时46分34秒
 ************************************************************************/
#ifndef __SERIAL_H_
#define __SERIAL_H_

//--------------------------- Define -----------------------------------//

//#define PORT_ADDR "/dev/ttyHSU1"
//#define BAUDRATE	115200
#define FALSE	-1
#define	TRUE	0
#define MILLION	1000000

//------------------------ Include Files -------------------------------//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <time.h>

//-------------------------- Typedef -----------------------------------//
//using namespace std;
class PortPoll
{
public:
	PortPoll(const char *port,int speed,int flow_ctrl,int databits,int stopbits,int parity):_speed(speed),_flow_ctrl(flow_ctrl),_databits(databits),_stopbits(stopbits),_parity(parity)
	{
		memset(_port,0,sizeof(_port));
		memcpy(_port,port,strlen(port));
		initialize(port,speed,flow_ctrl,databits,stopbits,parity);
		
	};
	~PortPoll();
	int initialize(const char *port,int speed,int flow_ctrl,int databits,int stopbits,int parity);
	int initialize(int speed)
	{
		_speed = speed;
		return initialize(_port,_speed,_flow_ctrl,_databits,_stopbits,_parity);
	}
	int initialize()
	{
		return initialize(_port,_speed,_flow_ctrl,_databits,_stopbits,_parity);
	}
	virtual int serial_get_one_frame(char *buf);
	int serial_read(char *buf,int cnt);
	int serial_write(char *buf,int cnt);
	int serial_close(void);
private:
	int _fd;
	char _port[20];
	int _speed;
	int _flow_ctrl;
	int _databits;
	int _stopbits;
	int _parity;
	
};

//-------------------------- Extern ------------------------------------//

#endif	//__serial_h__
