/*************************************************************************
	> File Name: _cpp
	> Author: Ev
	> Mail: wang2011yiwei@sina.com 
	> Created Time: 2017年03月13日 星期一 22时04分39秒
 ************************************************************************/

//------------------------ Include Files -------------------------------//

#include "serial.hpp"
#include <iostream>

using namespace std;
//--------------------- Function Prototype -----------------------------//

//int PortPoll_initialize(int speed);
//int PortPoll_read(char *buf);
//int PortPoll_write(char *buf,int cnt);

//--------------------------- Veriable ---------------------------------//

//PORT_T PortPoll = {
//	.initialize = PortPoll_initialize,
//	.read = PortPoll_read,
//	.write = PortPoll_write
//};

//------------------------- Function -----------------------------------//
PortPoll::~PortPoll()
{

	serial_close();
}

int PortPoll::initialize(const char *port,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int i;
    int speed_arr[] = {B921600,B500000,B460800, B230400, B115200,B57600, B19200, B9600, B4800, B2400, B1200, B300};
    int name_arr[] = {921600,500000,460800, 230400, 115200, 57600,  19200, 9600, 4800, 2400, 1200, 300};

    struct termios options;

    //_flow_ctrl = flow_ctrl;
    //_databits = databits;
    //_stopbits = stopbits;
    //_parity = parity;
	//_speed = speed;
 
    ////open port
	//memset(_port,0,sizeof(_port));
	//memcpy(_port,port,strlen(port));
    _fd = open(_port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (_fd == FALSE)
    {
        perror("Can't Open Serial Port");
        return (FALSE);
    }else printf("open serial ok !\n");
    if (fcntl(_fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed!\n");
        return (FALSE);
    }
    if (0 == isatty(STDIN_FILENO))
    {
        printf("standard input is not a terminal device\n");
        return (FALSE);
    }
    //set port

       //get parameter
    if (tcgetattr(_fd, &options) != 0)
    {
        perror("SetupSerial 1");
        return (FALSE);
    }
    //set input / output speed
    for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }
    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch (flow_ctrl)
    {
    case 0: //不使用流控制
        options.c_cflag &= ~CRTSCTS;
        break;
    case 1: //使用硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2: //使用软件流控制
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "Unsupported data size\n");
        return (FALSE);
    }
    //设置校验位
    switch (parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O': //设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E': //设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported parity\n");
        return (FALSE);
    }
    // 设置停止位
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported stop bits\n");
        return (FALSE);
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //我加的
                                                        //options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待50*(1/10)s */
    options.c_cc[VMIN] = 1;  /* 读取字符的最少个数为0 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(_fd, TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(_fd, TCSANOW, &options) != 0)
    {
        perror("com set error!\n");
        return (FALSE);
    }

    return (TRUE);
}

int PortPoll::serial_get_one_frame(char *buf)
{
    char c = 0;
    int err = 0;
    char *p = buf;

    while (1)
    {
        err = read(_fd, &c, 1);
        if (err < 1)
            return err - 1;
        //if (c != '\n'){
        //    *(p++) = c;
		//}
		if((*(p ++) = c) == '\n') break;
        //else
        //    break;
        if ((int)(p - buf) > 511){
			*p = '\0';
            return 10;
		}
    }
    *p = '\0';
    return 0;
}

int PortPoll::serial_read(char *buf, int count)
{
	return read(_fd,buf,count);
}
int PortPoll::serial_write(char *buf, int count)
{
	return write(_fd,buf,count);
    //int len = 0;
    //len = write(_fd, buf, count);
    //if (len == count)
    //{
    //    return 0;
    //}
    //else
    //{
    //    tcflush(_fd, TCOFLUSH);
    //    return FALSE;
    //}
}
int PortPoll::serial_close(void)
{
    printf("close serial ok!\n");
    return close(_fd);
    
}


