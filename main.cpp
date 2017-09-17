/*************************************************************************
	> File Name: main.cpp
	> Author: Ev
	> Mail: wang2011yiwei@sina.com 
	> Created Time: 2017年03月24日 星期五 11时09分21秒
 ************************************************************************/

//------------------------ Include Files -------------------------------//
#include "serial.hpp"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "mavlink_parse.hpp"
#include <thread>
#include <condition_variable>    // std::condition_variable
// #include <iostream>
#include <chrono>
#include <ratio>
#include <ctime>
#include <mutex>
#include <deque> 
#include <signal.h>
#include "Eigen/Geometry"

#include <GL/glew.h>        // GLEW扩展库
#include <GL/freeglut.h>  // freeGLUT图形库
// #include <GL/glut.h>
/* assimp include files. These three are usually needed. */
#include "assimp/cimport.h"
#include "assimp/scene.h"
#include "assimp/postprocess.h"

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
//--------------------------- Veriable ---------------------------------//
Mat ned_pic(400, 400, CV_8UC3);
/* the global Assimp scene object */
const struct aiScene* scene = NULL;
GLuint scene_list = 0;
aiVector3D scene_min, scene_max, scene_center;

/* current rotation angle */
static float angle = 0.f;

#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)
    static float xrotate = 0; 
    static float yrotate = 0; 
    static float zrotate = 0; 
// pthread_mutex_t mavlink_mutex; //互斥锁 

// std::mutex mavlink_mutex;
std::mutex mavlink_imu_mutex;
std::mutex mavlink_gps_status_mutex;
// std::mutex mavlink_gps_raw_mutex;
std::mutex mavlink_attitude_quaternion_mutex;
std::mutex count_mutex;
std::mutex pos_mutex;
std::mutex gps_raw_mutex;

std::condition_variable cond_v; // 全局条件变量.
std::condition_variable cond_v_pos; // 全局条件变量.
std::condition_variable cond_v_gps_raw; // 全局条件变量.

int count_105 = 0;

std::deque<mavlink_highres_imu_t> l_imuFromPX4;
std::deque<mavlink_gps_status_t> l_gpsStatusPx4;
std::deque<mavlink_gps_raw_int_t> l_gpsRawIntPx4;
std::deque<mavlink_attitude_quaternion_t> l_AttitudeQuaternionPx4;
std::deque<mavlink_local_position_ned_t> l_local_position_ned;

// MavlinkParse mavlink_message;
MavlinkParse *mavlink_message_p = new MavlinkParse;

int run_flag = 1;
Eigen::Quaternionf quat_temp;
typedef struct{
    double lat;
    double lng;
    double height;
}GPS_G_T;
double position[3];
Point2d center_p;
mavlink_gps_raw_int_t gps_raw_data;
//--------------------- Function Prototype -----------------------------//

// void * HandleMavlinkMessage(void *arg);
Eigen::Vector3f to_euler(Eigen::Quaternionf data);
Eigen::Vector4f to_rotate(Eigen::Quaternionf data);
void myDisplay(void);
void DrawCube(void);
void GetMavlinkData();
void display(void);
void reshape(int width, int height);
int loadasset (const char* path);
void processKeys(int key, int x, int y);
int draw_ned(Mat &pic, Point2d ned_data, Point2d gps_data, int flag);
Point conv_xy(Point2d p, double flag, Point2d center);
int gps2ned(mavlink_gps_raw_int_t *gps);
#define READ_ONLY 0
//------------------------- Function -----------------------------------//

void signhandler(int isignnum)
{
	delete mavlink_message_p;
	run_flag = 0;
	exit;
}

void LocalPositionNedAnalyze()
{
	using namespace std::chrono;
	int count = 0;
	steady_clock::time_point t1,t2;
	t1 = steady_clock::now();
	duration<double> time_span;
	char key = 0;
	int scal_p = 1;
	Point2d gps_data;
#if READ_ONLY
    
    std::ifstream infile;
    infile.open("save.txt");
#else
    std::ofstream outfile;
    outfile.open("save.txt");
#endif
	Point2d data;
    center_p.x = 0;
    center_p.y = 0;
	while(run_flag){

#if READ_ONLY
	if(infile >> data.x >> data.y >> gps_data.x >> gps_data.y){
		std::cout << "data: " << data << "\t gps_data: " << gps_data << std::endl;
		draw_ned(ned_pic, data, gps_data, scal_p);
		imshow("ned", ned_pic);
    }
    usleep(1000);
#else		
		std::unique_lock<std::mutex> lock(pos_mutex);

		// mavlink_attitude_quaternion_mutex.lock();
		while(l_local_position_ned.empty()){
			cond_v_pos.wait(lock);
		}
		while(!l_local_position_ned.empty()){
			count ++;
			data.x = l_local_position_ned.back().x;
			data.y = l_local_position_ned.back().y;
			// gps_data.x = position[0];
			// gps_data.y = position[1];
			l_local_position_ned.pop_back();
		}
		// mavlink_attitude_quaternion_mutex.unlock();
		lock.unlock();
		if(data.x != 0 && data.y != 0){
			gps2ned(&gps_raw_data);
			gps_data.x = position[0];
			gps_data.y = position[1];
			std::cout << "data: " << data << "\t gps_data: " << gps_data << std::endl;
			outfile << data.x << " " << data.y << " " << gps_data.x << " " << gps_data.y << "\n";
			draw_ned(ned_pic, data, gps_data, scal_p);
			imshow("ned", ned_pic);

		}
	#endif
		if (key == '='){
			scal_p++;
			if (scal_p > 10)
				scal_p = 10;
		}else if (key == '-'){
			scal_p--;
			if (scal_p < 0)
				scal_p = 0;
		}else if (key == 'w'){
			center_p.y++;
		}else if (key == 's'){
			center_p.y--;
		}else if (key == 'a'){
			center_p.x--;
		}else if (key == 'd'){
			center_p.x++;
		}

		key = waitKey(100);
		
		t2 = steady_clock::now();
		time_span = duration_cast<duration<double>>(t2 - t1);

		if(time_span.count() > 0.5){
			count = 0;
			//std::cout << "hello" << std::endl;
			t1 = steady_clock::now();
		}

	}
	#if READ_ONLY
		infile.close();
	#else
		outfile.close();
	#endif
	pthread_exit(0);
}

void AttitudeQuaternionAnalyze()
{
	using namespace std::chrono;
	int count = 0;
	steady_clock::time_point t1,t2;
	t1 = steady_clock::now();
	duration<double> time_span;

	// float quaternion[4];
	float data[4];
	Eigen::VectorXf quanternion(4);
	Eigen::Quaternionf attitude_quanternion;
	Eigen::AngleAxis<float> attitude_angle;

	while(run_flag){
		std::unique_lock<std::mutex> lock(mavlink_attitude_quaternion_mutex);

		// mavlink_attitude_quaternion_mutex.lock();
		while(l_AttitudeQuaternionPx4.empty()){
			cond_v.wait(lock);
		}
		while(!l_AttitudeQuaternionPx4.empty()){
			count ++;
			
			attitude_quanternion.w() = l_AttitudeQuaternionPx4.back().q1;
			attitude_quanternion.x() = l_AttitudeQuaternionPx4.back().q2;
			attitude_quanternion.y() = l_AttitudeQuaternionPx4.back().q3;
			attitude_quanternion.z() = l_AttitudeQuaternionPx4.back().q4;
			
			// data[0] = l_AttitudeQuaternionPx4.back().q1;
			// data[1] = l_AttitudeQuaternionPx4.back().q2;
			// data[2] = l_AttitudeQuaternionPx4.back().q3;
			// data[3] = l_AttitudeQuaternionPx4.back().q4;
			attitude_angle = attitude_quanternion;
			quat_temp  = attitude_quanternion;
			l_AttitudeQuaternionPx4.pop_back();
		}
		// mavlink_attitude_quaternion_mutex.unlock();
		lock.unlock();

		t2 = steady_clock::now();
		time_span = duration_cast<duration<double>>(t2 - t1);

		if(time_span.count() > 0.5){
			// myDisplay();

			// printf("\033c ");
			// t1 = steady_clock::now();
			Eigen::Vector3f vv = to_euler(attitude_quanternion);
			// t2 = steady_clock::now();
			// time_span = duration_cast<duration<double>>(t2 - t1);		
			// printf("q[0]: %f,q[1]: %f,q[2]: %f,q[3]: %f ",quaternion[0],quaternion[1],quaternion[2],quaternion[3]);
			// std::cout << "quaternion frequency: " << count << std::endl;
			// Eigen::Vector3f vv(attitude_quanternion.toRotationMatrix().eulerAngles(0,1,2));
			// if(v[0] > M_PI/2)V[0] -= M_PI;
			// std::cout << "time used :" << time_span.count() << std::endl;
			// printf("time used : %f \n",time_span.count());
			// std::cout << "roll  is : " << vv(0) * 180 / M_PI<< std::endl;
			// // if(v[1] > M_PI/2)V[1] -= M_PI;
			// std::cout << "pitch is : " << vv(1) * 180 / M_PI<< std::endl;
			// // if(v[2] > M_PI/2)V[2] -= M_PI;
			// std::cout << "yaw   is : " << vv(2) * 180 / M_PI<< std::endl;
			// std::cout << "\nangle is : \n" << attitude_angle.matrix().eulerAngles(0,1,2) << std::endl;
			count = 0;
			t1 = steady_clock::now();
		}

	}
	pthread_exit(0);
}

void ImuAnalyze()
{
	int count = 0;
	std::chrono::steady_clock::time_point t1,t2;
	t1 = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_span;

	float xgyro_get = 0;
	float ygyro_get = 0;
	float zgyro_get = 0;
	int64_t time_esp = 0;
	int64_t time_bak = 0;

	float current_xgyro = 0;
	float current_ygyro = 0;
	float current_zgyro = 0;


	while(run_flag){

		mavlink_imu_mutex.lock();
		while(!l_imuFromPX4.empty()){
			count ++;
			xgyro_get = l_imuFromPX4.back().xgyro;
			ygyro_get = l_imuFromPX4.back().ygyro;
			zgyro_get = l_imuFromPX4.back().zgyro;
			time_esp = l_imuFromPX4.back().time_usec;
			l_imuFromPX4.pop_back();

		// 			//处理得到的imu数据
		{
			if(time_bak != 0){
				current_xgyro += xgyro_get * (time_esp - time_bak)*0.000001;
				current_ygyro += ygyro_get * (time_esp - time_bak)*0.000001;
				current_zgyro += zgyro_get * (time_esp - time_bak)*0.000001;
				// std::cout << "imu x: " << current_xgyro << std::endl;
			}
			time_bak = time_esp;
			
			
		}
		}
		mavlink_imu_mutex.unlock();

		t2 = std::chrono::steady_clock::now();
		time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

		if(time_span.count() > 1){
			// std::cout << "imu frequency: " << count << std::endl;
			// std::cout << "imu x: " << current_xgyro << " imu y: " << current_ygyro << " imu z: " << current_zgyro << std::endl;
			count = 0;
			t1 = std::chrono::steady_clock::now();
		}

	}
	pthread_exit(0);

}

void GpsRawInitAnalyze()
{
	using namespace std::chrono;
	int count = 0;
	steady_clock::time_point t1,t2;
	t1 = steady_clock::now();
	duration<double> time_span;

	while(run_flag){
		std::unique_lock<std::mutex> lock(gps_raw_mutex);
		while(l_gpsRawIntPx4.empty()){
			cond_v_gps_raw.wait(lock);
		}
		// mavlink_gps_raw_mutex.lock();
		while(!l_gpsRawIntPx4.empty()){
			count ++;
			gps_raw_data = l_gpsRawIntPx4.back();
			l_gpsRawIntPx4.pop_back();
		}
		// mavlink_gps_raw_mutex.unlock();
		lock.unlock();

		t2 = steady_clock::now();
		time_span = duration_cast<duration<double>>(t2 - t1);

		if(time_span.count() > 1){
			// std::cout << "gps_raw frequency: " << count << std::endl;
			count = 0;
			t1 = steady_clock::now();
		}

	}
	pthread_exit(0);
}

void GpsStatusAnalyze()
{
	using namespace std::chrono;
	int count = 0;
	steady_clock::time_point t1,t2;
	t1 = steady_clock::now();
	duration<double> time_span;
	while(run_flag){

		mavlink_gps_status_mutex.lock();
		while(!l_gpsStatusPx4.empty()){
			count ++;
			l_gpsStatusPx4.pop_back();
		}
		mavlink_gps_status_mutex.unlock();

		t2 = steady_clock::now();
		time_span = duration_cast<duration<double>>(t2 - t1);

		if(time_span.count() > 1){
			// std::cout << "gps_status frequency: " << count << std::endl;
			count = 0;
			t1 = steady_clock::now();
		}

	}
	pthread_exit(0);
}

	//按键输入处理回调函数  
void processKeys(int key, int x, int y) {  
	
	if(key==GLUT_KEY_UP){  
		xrotate-=0.1f;  
		
	}  
	else if(key==GLUT_KEY_DOWN){  
		xrotate+=0.1f;  
	}  
	else if(key==GLUT_KEY_LEFT){  
		yrotate-=0.1f;  
	}  
	else if(key==GLUT_KEY_RIGHT){  
		yrotate+=0.1f;  
	}
	else if(key==GLUT_KEY_F1){
		zrotate-=0.1f;
	}
	else if(key==GLUT_KEY_F2){
		zrotate+=0.1f;
	}  
	//重新绘制  
	// glutPostRedisplay();  
} 
void OnTimer(int value)
{
//    alpha++;
//    alpha=(alpha%256);
   glutPostRedisplay();
   glutTimerFunc(33, OnTimer, 1);
}
int main(int argc, char **argv)
{
	std::thread threads[6];

	threads[0] = std::thread(GetMavlinkData);
	// threads[1] = std::thread(ImuAnalyze);
	threads[2] = std::thread(GpsRawInitAnalyze);
	// threads[3] = std::thread(GpsStatusAnalyze);
	threads[4] = std::thread(AttitudeQuaternionAnalyze);
	threads[5] = std::thread(LocalPositionNedAnalyze);

	char get_buf[64];
	int get_number = 0;

	signal(SIGINT,signhandler);
/* opengl  */

      	glutInit(&argc, argv);  
        glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);  
        glutInitWindowPosition(100,100);  
        glutInitWindowSize(900,600);
        glutCreateWindow("GLDemo");  
		glEnable(GL_DEPTH_TEST);
       // glutReshapeFunc(changSize);
        // glutDisplayFunc(&myDisplay);
        // glutSpecialFunc(specialKey);
		// glutIdleFunc(display);
		glutTimerFunc(33, OnTimer, 1);
		glutDisplayFunc(display);
		glutReshapeFunc(reshape);
	/* the model name can be specified on the command line. If none
	  is specified, we try to locate one of the more expressive test 
	  models from the repository (/models-nonbsd may be missing in 
	  some distributions so we need a fallback from /models!). */
	  
	if( 0 != loadasset( argc >= 2 ? argv[1] : "../fly.3DS")) {
		goto end;
		// if( argc != 1 || (0 != loadasset( "../../../../test/models-nonbsd/X/dwarf.x") && 0 != loadasset( "../../test/models/X/Testwuson.X"))) { 
		// 	return -1;
		// }
	}

	glClearColor(0.1f,0.1f,0.1f,1.f);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);    /* Uses default lighting parameters */
	// GLfloat light_ambient[] = { 0, 1, 1, 1 }; 
	// glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);//设置光的环境强度 
	glEnable(GL_DEPTH_TEST);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glEnable(GL_NORMALIZE);

	/* XXX docs say all polygons are emitted CCW, but tests show that some aren't. */
	if(getenv("MODEL_IS_BROKEN"))  
		glFrontFace(GL_CW);

	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

	glutGet(GLUT_ELAPSED_TIME);

	glutSpecialFunc(processKeys);
	glutMainLoop();

	/* cleanup - calling 'aiReleaseImport' is important, as the library 
	   keeps internal resources until the scene is freed again. Not 
	   doing so can cause severe resource leaking. */
	aiReleaseImport(scene); 
    // glutInit(&argc, argv);

    // glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);

    // glutInitWindowPosition(100, 100);

    // glutInitWindowSize(400, 400);

    // glutCreateWindow("第一个OpenGL程序");

    // init();

    // glutDisplayFunc(&myDisplay);
	
	// glutIdleFunc(&myIdle);	

    // glutMainLoop();
/* opengl  */
	// while(run_flag){
	// 	// get_number = mavlink_message_p->serial->serial_read(get_buf,64);
	// 	// for(int i = 0; i < get_number; i ++){
	// 	// 	printf("0x%02x ",(unsigned char)get_buf[i]);
	// 	// }
	// 	// printf("\n");
	// 	sleep(1);
	// 	// count_mutex.lock();
	// 	// std::cout << "count 105: " << count_105 << std::endl;
	// 	// count_105 = 0;
	// 	// count_mutex.unlock();

	// }
end:
delete mavlink_message_p;
	run_flag = 0;
	for(auto &th:threads) th.join();
		// pthread_t ntid;
		// void *retval;
		// int err;
		// int imu = 0;
		// int gpsraw = 0;
		// int gpsstatus = 0;

		// // pthread_mutex_init(&mavlink_mutex,NULL); //对锁进行初始化 

		// err = pthread_create(&ntid, NULL,HandleMavlinkMessage,NULL);

		// if(err != 0){
		// 	printf("can't create thread: %s\n",strerror(err));
		// }
		// 	while(1){
		// 		sleep(1);
		// 		printf("\033c\n");
		// 		// pthread_mutex_lock(&mavlink_mutex);

		// 		imu = mavlink_message.ImuCount ;
		// 		gpsraw = mavlink_message.GpsRawInitCount;
		// 		gpsstatus = mavlink_message.GpsStatusCount;

		// 		mavlink_message.ImuCount = 0;
		// 		mavlink_message.GpsRawInitCount = 0;
		// 		mavlink_message.GpsStatusCount = 0;
				
				
		// 		// pthread_mutex_unlock(&mavlink_mutex);

		// 		printf("imu frequency: %d\n",imu);
		// 		printf("GpsRawInit frequency: %d\n",gpsraw);
		// 		printf("GpsStatus frequency: %d\n",gpsstatus);

		// 	}
		// 	fd = UART0_Open(fd, "/dev/ttyUSB0"); //打开串口，返回文件描述符
		// 	do {
		// 		err = UART0_Init(fd, 230400, 0, 8, 1, 'N');
		// 	   printf("Set Port Exactly!\n");
		// 	} while (-1 == err || -1 == fd);

		// 	while(1){

		// 	//	i ++;
		// 	//	err = write(fd,&i,1);
		// 	//	err = read(fd,&c,1);
		// 	//	printf("%d\n",c);
		// 	read(fd,&c,1);
		// 	get_c = (unsigned char)c;
		// 	if(get_c == 0xfe){
		// 		i = 0;
		// //		fputc('\n',stdout);
		// 		printf("\n ");
		// 	}else i ++;
		// 	if(i == 5){
		// 		//time (&timep);
		// 		//printf("%03d   %s\n",get_c,ctime(&timep));
		// 		printf("\033[32m%02x \033[39m",get_c);
		// 	}else
		// 	//usleep(10000);
		// 	//fputc(c,stdout);
		// 	printf("%02x ",get_c);
			
		// 	//fflush(stdout);

		// 	}
		// 	pthread_join(ntid,&retval);
	return 0;
}

void GetMavlinkData()
{
	// mavlink_message.ImuCount = 0;
	// mavlink_message.GpsRawInitCount = 0;
	// mavlink_message.GpsStatusCount = 0;
	int flag_count = 0;
	mavlink_highres_imu_t scaled_imu;
	mavlink_gps_status_t gps_status;
	mavlink_gps_raw_int_t gps_raw_init;	
	mavlink_attitude_quaternion_t attitude_quaternion;
	mavlink_local_position_ned_t pos;

	mavlink_message_t message;

	mavlink_status_t status;
	unsigned int decodeState = 0;
	char get_buf[64];
	
	int get_number = 0;

	// uint8_t temp_buf[1024];//
	// int temp_length = 0;//
	// int temp_flag = 0;//
	// for(int i = 0; i < 1000; i ++){
	// 	get_number = mavlink_message.serial->serial_read(get_buf,64);
	// }
	while(run_flag){
		// if(mavlink_message.get_message(message) == 0){
		// 	std::cout << "read mavlink message error" << std::endl;
		// } 
		get_number = mavlink_message_p->serial->serial_read(get_buf,64);

		if(get_number < 1){
			std::cout << "read error" << std::endl;
			continue;
		}
	
		for(int i = 0; i < get_number; i ++){

			decodeState = mavlink_parse_char(0, (uint8_t)(get_buf[i]), &message, &status);

			// if((uint8_t)get_buf[i] == 0xfe){
			
			// 	flag_count = 0;
		
			// }
			// flag_count ++;

			// if(flag_count == 6 && get_buf[i] == MAVLINK_MSG_ID_ATTITUDE_QUATERNION){
				
			
			// 	count_mutex.lock();
				
			// 	count_105 ++;
			// 	count_mutex.unlock();
			// }

			if(decodeState == 1){
				
		// 		if(message.msgid == MAVLINK_MSG_ID_HIGHRES_IMU){
					
		// 			// printf("imu frequency: %d\n",mavlink_message.ImuCount);
					
		// 			// mavlink_message.ImuCount ++;
		// 			mavlink_message_p->handlemavlinkScaledImu(message,scaled_imu);
					
		// 			mavlink_imu_mutex.lock();
		// 			l_imuFromPX4.push_back(scaled_imu);
		// 			mavlink_imu_mutex.unlock();
			
		// //			// l_imuFromPX4.push_back(highres_imu);
				
		// 			// cout << "get" << endl;
		// 			// printf("imu.xacc: %f\t imu.yacc: %f\t imu.zacc: %f\n",mavlink_message.scaled_imu.xacc,mavlink_message.scaled_imu.yacc,mavlink_message.scaled_imu.zacc);
		// 			// printf("imu.xgyro: %f\t imu.ygyro: %f\t imu.zgyro: %f\n",mavlink_message.scaled_imu.xgyro,mavlink_message.scaled_imu.ygyro,mavlink_message.scaled_imu.zgyro);
		// 			// printf("imu.xmag: %f\t imu.ymag: %f\t imu.zmag: %f\n",mavlink_message.scaled_imu.xmag,mavlink_message.scaled_imu.ymag,mavlink_message.scaled_imu.zmag);

		// 		}else 
				if(message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT){
					
					// printf("GpsRawInit frequency: %d\n",mavlink_message.GpsRawInitCount);

					// mavlink_message.GpsRawInitCount ++;
					mavlink_message_p->handlemavlinkGpsRawInit(message,gps_raw_init);
					// mavlink_gps_raw_mutex.lock();
					std::unique_lock <std::mutex> lck(gps_raw_mutex);
					l_gpsRawIntPx4.push_back(gps_raw_init);
					// std::cout << "gps_raw_init.lat ：" << gps_raw_init.lat << std::endl;
					cond_v_gps_raw.notify_all(); // 唤醒所有线程.
					// .unlock();
					lck.unlock();

				}else if(message.msgid == MAVLINK_MSG_ID_GPS_STATUS){
					
					// printf("GpsStatus frequency: %d\n",mavlink_message.GpsStatusCount);

					// mavlink_message.GpsStatusCount ++;
					mavlink_message_p->handlemavlinkGpsStatus(message,gps_status);
					// std::cout << "gps status count" << std::endl;
					mavlink_gps_status_mutex.lock();
					
					l_gpsStatusPx4.push_back(gps_status);

					mavlink_gps_status_mutex.unlock();
					

				}else if(message.msgid == MAVLINK_MSG_ID_ATTITUDE_QUATERNION){

					mavlink_message_p->handlemavlinkAttitudeQuaternion(message,attitude_quaternion);
					
					// mavlink_attitude_quaternion_mutex.lock();
					std::unique_lock <std::mutex> lck(mavlink_attitude_quaternion_mutex);
					l_AttitudeQuaternionPx4.push_back(attitude_quaternion);
					cond_v.notify_all(); // 唤醒所有线程.
					// mavlink_attitude_quaternion_mutex.unlock();
					lck.unlock();

				}else if(message.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED){
					mavlink_message_p->handlemavlinkLocalPosition(message,pos);
					// .lock();
					std::unique_lock <std::mutex> lck(pos_mutex);
					l_local_position_ned.push_back(pos);
					cond_v_pos.notify_all(); // 唤醒所有线程.
					// .unlock();
					lck.unlock();	
				}
			}
		}
	}
	
	pthread_exit(0);
}

Eigen::Vector3f to_euler(Eigen::Quaternionf data){
	data.normalized();
	return Eigen::Vector3f(
		atan2f(2.0f * (data.w() * data.x() + data.y() * data.z()), 1.0f - 2.0f * (data.x() * data.x() + data.y() * data.y())),
		asinf(2.0f * (data.w() * data.y() - data.z() * data.x())),
		atan2f(2.0f * (data.w() * data.z() + data.x() * data.y()), 1.0f - 2.0f * (data.y() * data.y() + data.z() * data.z()))
	);
}

Eigen::Quaternionf from_euler(Eigen::Vector3f vector) {
	double cosPhi_2 = cos(double(vector(0)) / 2.0);
	double sinPhi_2 = sin(double(vector(0)) / 2.0);
	double cosTheta_2 = cos(double(vector(1)) / 2.0);
	double sinTheta_2 = sin(double(vector(1)) / 2.0);
	double cosPsi_2 = cos(double(vector(2)) / 2.0);
	double sinPsi_2 = sin(double(vector(2)) / 2.0);

	/* operations executed in double to avoid loss of precision through
		* consecutive multiplications. Result stored as float.
		*/
	Eigen::Quaternionf data;
	data.w() = static_cast<float>(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);
	data.x() = static_cast<float>(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);
	data.y() = static_cast<float>(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);
	data.z() = static_cast<float>(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
	data.normalized();
	return data;
}

Eigen::Vector4f to_rotate(Eigen::Quaternionf data){
	
	float theta = acosf(data.w()) * ((data.w() > 0) ? 1:-1 );
	return Eigen::Vector4f(
		theta * 2 /M_PI * 180,
		data.x() / (sin(theta)),
		data.y() / (sin(theta)),
		data.z() / (sin(theta))
	);
}


void reshape(int width, int height)
{
	const double aspectRatio = (float) width / height, fieldOfView = 45.0;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fieldOfView, aspectRatio,
		1.0, 1000.0);  /* Znear and Zfar */
	glViewport(0, 0, width, height);
}

/* ---------------------------------------------------------------------------- */
void get_bounding_box_for_node (const struct aiNode* nd, 
	aiVector3D* min, 
	aiVector3D* max, 
	aiMatrix4x4* trafo
){
	aiMatrix4x4 prev;
	unsigned int n = 0, t;

	prev = *trafo;
	aiMultiplyMatrix4(trafo,&nd->mTransformation);

	for (; n < nd->mNumMeshes; ++n) {
		const struct aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
		for (t = 0; t < mesh->mNumVertices; ++t) {

			aiVector3D tmp = mesh->mVertices[t];
			aiTransformVecByMatrix4(&tmp,trafo);

			min->x = aisgl_min(min->x,tmp.x);
			min->y = aisgl_min(min->y,tmp.y);
			min->z = aisgl_min(min->z,tmp.z);

			max->x = aisgl_max(max->x,tmp.x);
			max->y = aisgl_max(max->y,tmp.y);
			max->z = aisgl_max(max->z,tmp.z);
		}
	}

	for (n = 0; n < nd->mNumChildren; ++n) {
		get_bounding_box_for_node(nd->mChildren[n],min,max,trafo);
	}
	*trafo = prev;
}

/* ---------------------------------------------------------------------------- */
void get_bounding_box (aiVector3D* min, aiVector3D* max)
{
	aiMatrix4x4 trafo;
	aiIdentityMatrix4(&trafo);

	min->x = min->y = min->z =  1e10f;
	max->x = max->y = max->z = -1e10f;
	get_bounding_box_for_node(scene->mRootNode,min,max,&trafo);
}

/* ---------------------------------------------------------------------------- */
void color4_to_float4(const aiColor4D *c, float f[4])
{
	f[0] = c->r;
	f[1] = c->g;
	f[2] = c->b;
	f[3] = c->a;
}

/* ---------------------------------------------------------------------------- */
void set_float4(float f[4], float a, float b, float c, float d)
{
	f[0] = a;
	f[1] = b;
	f[2] = c;
	f[3] = d;
}

/* ---------------------------------------------------------------------------- */
void apply_material(const struct aiMaterial *mtl)
{
	float c[4];

	GLenum fill_mode;
	int ret1, ret2;
	aiColor4D diffuse;
	aiColor4D specular;
	aiColor4D ambient;
	aiColor4D emission;
	float shininess, strength;
	int two_sided;
	int wireframe;
	unsigned int max;

	set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
		color4_to_float4(&diffuse, c);
		// printf("color: %f\t %f\t %f\t %f\n",c[0],c[1],c[2],c[3]);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);

	set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular)){
		color4_to_float4(&specular, c);
		// printf("color: %f\t %f\t %f\t %f\n",c[0],c[1],c[2],c[3]);
	}
		
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);

	set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
		color4_to_float4(&ambient, c);
		// printf("color: %f\t %f\t %f\t %f\n",c[0],c[1],c[2],c[3]);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);

	set_float4(c, 0.2f, 0.20f, 0.20f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
		color4_to_float4(&emission, c);
		// printf("color: %f\t %f\t %f\t %f\n",c[0],c[1],c[2],c[3]);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);

	max = 1;
	ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
	if(ret1 == AI_SUCCESS) {
    	max = 1;
    	ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
		if(ret2 == AI_SUCCESS)
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
        else
        	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
    }
	else {
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
		set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
	}

	max = 1;
	if(AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
		fill_mode = wireframe ? GL_LINE : GL_FILL;
	else
		fill_mode = GL_FILL;
	glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

	max = 1;
	if((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && two_sided)
		glDisable(GL_CULL_FACE);
	else 
		glEnable(GL_CULL_FACE);
}

/* ---------------------------------------------------------------------------- */
void recursive_render (const struct aiScene *sc, const struct aiNode* nd)
{
	unsigned int i;
	unsigned int n = 0, t;
	
	aiMatrix4x4 m = nd->mTransformation;

	/* update transform */
	aiTransposeMatrix4(&m);
	glPushMatrix();
	glMultMatrixf((float*)&m);

	/* draw all meshes assigned to this node */
	for (; n < nd->mNumMeshes; ++n) {
		const struct aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];

		apply_material(sc->mMaterials[mesh->mMaterialIndex]);

		if(mesh->mNormals == NULL) {
			glDisable(GL_LIGHTING);
		} else {
			glEnable(GL_LIGHTING);
		}

		for (t = 0; t < mesh->mNumFaces; ++t) {
			const struct aiFace* face = &mesh->mFaces[t];
			GLenum face_mode;

			switch(face->mNumIndices) {
				case 1: face_mode = GL_POINTS; break;
				case 2: face_mode = GL_LINES; break;
				case 3: face_mode = GL_TRIANGLES; break;
				default: face_mode = GL_POLYGON; break;
			}

			glBegin(face_mode);

			for(i = 0; i < face->mNumIndices; i++) {
				int index = face->mIndices[i];
				if(mesh->mColors[0] != NULL)
					glColor4fv((GLfloat*)&mesh->mColors[0][index]);
				if(mesh->mNormals != NULL) 
					glNormal3fv(&mesh->mNormals[index].x);
				glVertex3fv(&mesh->mVertices[index].x);
			}

			glEnd();
		}

	}

	/* draw all children */
	for (n = 0; n < nd->mNumChildren; ++n) {
		recursive_render(sc, nd->mChildren[n]);
	}

	glPopMatrix();
}

/* ---------------------------------------------------------------------------- */
void do_motion (void)
{
	static GLint prev_time = 0;
	static GLint prev_fps_time = 0;
	static int frames = 0;

	int time = glutGet(GLUT_ELAPSED_TIME);
	angle += (time-prev_time)*0.01;
	prev_time = time;

	frames += 1;
	if ((time - prev_fps_time) > 1000) /* update every seconds */
    {
        int current_fps = frames * 1000 / (time - prev_fps_time);
        printf("%d fps\n", current_fps);
		printf("rotate %f/t  %f/t %f/t\n",xrotate,yrotate,zrotate);
        frames = 0;
        prev_fps_time = time;
    }


	// glutPostRedisplay ();
}

/* ---------------------------------------------------------------------------- */
void display(void)
{
	float tmp;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	Eigen::Vector4f rotate_temp = to_rotate(quat_temp);
	//777.blend
	// gluLookAt(0.0f,-2.0f,0.0f,0.f,0.f,0.f,0.f,0.f,1.f);
	// glRotatef(rotate_temp(0), -rotate_temp(1), rotate_temp(2),-rotate_temp(3));
	// glRotatef(-90,0,0,1);
	//fly.3ds
	gluLookAt(0.0f,-2.0f,0.0f,0.f,0.f,0.f,0.f,0.f,-1.f);
	glRotatef(rotate_temp(0), rotate_temp(1), rotate_temp(2),rotate_temp(3));
	glRotatef(-90,1,0,0);
	// gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0);
		// Eigen::Vector3f vv = to_euler(quat_temp);
		// glRotatef(vv(0) * 180 / M_PI,1,0,0);
		// glRotatef(vv(1) * 180 / M_PI,0,1,0);
		// glRotatef(vv(2) * 180 / M_PI,0,0,1);
	/* rotate it around the y axis */
	// glRotatef(angle,0.f,1.f,0.f);
		
		// std::cout << "roate: " << rotate_temp(0) << rotate_temp(1) << rotate_temp(2) << rotate_temp(3) << std::endl;

	/* scale the whole asset to fit into our view frustum */
	tmp = scene_max.x-scene_min.x;
	tmp = aisgl_max(scene_max.y - scene_min.y,tmp);
	tmp = aisgl_max(scene_max.z - scene_min.z,tmp);
	tmp = 1.f / tmp;
	glScalef(tmp, tmp, tmp);

        /* center the model */
	glTranslatef( -scene_center.x, -scene_center.y, -scene_center.z );

        /* if the display list has not been made yet, create a new one and
           fill it with scene contents */
	if(scene_list == 0) {
	    scene_list = glGenLists(1);
	    glNewList(scene_list, GL_COMPILE);
            /* now begin at the root node of the imported data and traverse
               the scenegraph by multiplying subsequent local transforms
               together on GL's matrix stack. */
	    recursive_render(scene, scene->mRootNode);
	    glEndList();
	}

	glCallList(scene_list);
 	glFlush();
	glutSwapBuffers();

	//do_motion();
}

/* ---------------------------------------------------------------------------- */
int loadasset (const char* path)
{
	/* we are taking one of the postprocessing presets to avoid
	   spelling out 20+ single postprocessing flags here. */
	//    std::cout << "hello "  << path << std::endl;
	scene = aiImportFile(path,aiProcessPreset_TargetRealtime_MaxQuality);

	if (scene) {
		get_bounding_box(&scene_min,&scene_max);
		scene_center.x = (scene_min.x + scene_max.x) / 2.0f;
		scene_center.y = (scene_min.y + scene_max.y) / 2.0f;
		scene_center.z = (scene_min.z + scene_max.z) / 2.0f;
		return 0;
	}
	return 1;
}

int draw_ned(Mat &pic, Point2d ned_data, Point2d gps_data,int flag)
{
	static int data_length;
	static Point2d data[4096];
	static Point2d gpsdata[4096];
    int wn = 512;
    int i;
    int scalar_data[10] = {
        2, 4, 8, 16, 32, 64, 128, 256, 512, 1024};
    pic = Mat::zeros(wn + 20, wn + 20, CV_8UC3);
    if(data_length < 4096){
        data[data_length ] = ned_data;
        gpsdata[data_length ++] = gps_data;

    }else{
        for(i = 0; i < 4096 - 1; i ++){
            data[i] = data[i + 1];
            gpsdata[i] = gpsdata[i + 1];
        }
        data[4096-1]= ned_data;
        gpsdata[4096-1]= gps_data;
    }

	//std::cout << "ned_data: " << ned_data << std::endl;
    //    line(parameter_image, pos_pre,pos_current,Scalar(255,0,0),1,8,0);
    line(pic, Point(0, wn / 2), Point(wn, wn / 2), Scalar(0, 100, 0), 1, 4, 0);
    line(pic, Point(wn / 2, 0), Point(wn / 2, wn), Scalar(0, 100, 0), 1, 4, 0);
    circle(pic, Point(wn / 2, wn / 2), wn / 4, Scalar(0, 100, 0), 1, 4);
    circle(pic, Point(wn / 2, wn / 2), wn / 2, Scalar(0, 100, 0), 1, 4);
    char temp[20];
    memset(temp, 0, sizeof(temp));
    sprintf(temp, "%4.2f", (double)(scalar_data[flag]));
    putText(pic, temp, Point(wn / 4 + wn / 2 - 20, wn / 2 + 20), 1, 1, Scalar(0, 100, 0), 1, 8, 0);
    memset(temp, 0, sizeof(temp));
    sprintf(temp, "%4.2f", (double)(scalar_data[flag + 1]));
    putText(pic, temp, Point(wn - 30, wn / 2 + 20), 1, 1, Scalar(0, 100, 0), 1, 8, 0);

    putText(pic, "N", Point(wn / 2, 20), 1, 1, Scalar(0, 100, 0), 1, 8, 0);
    putText(pic, "S", Point(wn / 2, wn - 10), 1, 1, Scalar(0, 100, 0), 1, 8, 0);
    putText(pic, "W", Point(10, wn / 2 + 20), 1, 1, Scalar(0, 100, 0), 1, 8, 0);
    putText(pic, "E", Point(wn - 30, wn / 2 - 10), 1, 1, Scalar(0, 100, 0), 1, 8, 0);

    Point2d posi_temp;
    for (i = 0; i < data_length; i++)
    {
        posi_temp = conv_xy(data[i], scalar_data[flag], data[data_length - 1]);
        if(posi_temp.x > wn || posi_temp.x < 0 || posi_temp.y > wn || posi_temp.y < 0){

		}else circle(pic, posi_temp, 2, Scalar(0, 0, 255), -1, 8, 0);

        posi_temp = conv_xy(gpsdata[i], scalar_data[flag], data[data_length - 1]);
        if(posi_temp.x > wn || posi_temp.x < 0 || posi_temp.y > wn || posi_temp.y < 0){
			
		}else circle(pic, posi_temp, 2, Scalar(0, 255, 0), -1, 8, 0);

    }

    return 0;
}

Point conv_xy(Point2d p, double flag, Point2d center)
{
    Point temp;
    temp.x = (p.x - center_p.x) / flag * 128 + 256;
    temp.y = -(p.y - center_p.y) / flag * 128 + 256;
    return temp;
    //    return (x - center_x)/flag *128 + 256;
}


int g2ecef(GPS_G_T *g_position,double *pe)
{
    //double lat = ((int)(lati/100)+(lati -(int)(lati/100))/60.)/180*PI;
    //double lng = ((int)(longi/100)+(longi -(int)(longi/100))/60.)/180*PI;
	const double L_AXIS = 6378137;
	const double F=0.003325810664;
	const double S_AXIS=L_AXIS*(1-F);

    double E_L  = (sqrt(((L_AXIS*L_AXIS) - (S_AXIS * S_AXIS))/(L_AXIS*L_AXIS)));
    double  E_S = (sqrt(((L_AXIS*L_AXIS) - (S_AXIS * S_AXIS))/(S_AXIS*S_AXIS)));
    double N = L_AXIS/sqrt(1-E_L*E_L*sin(g_position->lat)*sin(g_position->lat));

    *pe = (N + g_position->height)*cos(g_position->lat)*cos(g_position->lng);
    *(pe + 1) = (N + g_position->height)*cos(g_position->lat)*sin(g_position->lng);
    *(pe + 2) = (N*(1-pow(E_L,2)) + g_position->height)*sin(g_position->lat);

    return 0;

}

int gps2g(mavlink_gps_raw_int_t *gps,GPS_G_T *g_position)
{
    g_position->lat = (double)gps->lat*1e-7/180*M_PI;
    g_position->lng = (double)gps->lon*1e-7/180*M_PI;
    g_position->height = gps->alt;

    return 0;
}


int ecef2ned(GPS_G_T *current_position,GPS_G_T *ref_position,double *ned)
{
    double r_n2e[3][3];
    int i,j;
    double pe[3];
    double pn_temp[3];
    double pn_pe_temp[3];
    double temp;

    r_n2e[0][0] = -sin(ref_position->lat)*cos(ref_position->lng);
    r_n2e[0][1] = -sin(ref_position->lat)*sin(ref_position->lng);
    r_n2e[0][2] = cos(ref_position->lat);
    r_n2e[1][0] = -sin(ref_position->lng);
    r_n2e[1][1] = cos(ref_position->lng);
    r_n2e[1][2] = 0;
    r_n2e[2][0] = -cos(ref_position->lat)*cos(ref_position->lng);
    r_n2e[2][1] = -cos(ref_position->lat)*sin(ref_position->lng);
    r_n2e[2][2] = -sin(ref_position->lat);

    g2ecef(ref_position,pe);
    g2ecef(current_position,pn_temp);

    for(i = 0; i < 3; i++){
        pn_pe_temp[i] = *(pn_temp+i) - *(pe + i);
    }

    for(i = 0; i < 3; i ++){
        temp = 0;
        for(j = 0;j < 3; j ++){
            temp += pn_pe_temp[j]*r_n2e[i][j];
        }
        *(ned + i) = temp;

    }
    return 0;
}

int gps2ned(mavlink_gps_raw_int_t *gps)
{
    static int flag = 0;
    static GPS_G_T ref_position;
    GPS_G_T current_position;

    //    int line_cnt = 0;

    if(flag == 0){
        if(gps->lon!= 0 || gps->lat!= 0){
            flag = 1;
            gps2g(gps,&ref_position);
        }

    }else{
        gps2g(gps,&current_position);
        ecef2ned(&current_position,&ref_position,position);
    }

//    printf("position: %f %f %f\n",position[0],position[1],position[2]);

//    memset(buf_temp,0,sizeof(buf_temp));
//    sprintf(buf_temp,"%03d: ",line_num ++);
//    fwrite(buf_temp,1,strlen(buf_temp),fpned);
//
//    memset(buf_temp,0,sizeof(buf_temp));
//    sprintf(buf_temp,"%f %f %f\n",position[0],position[1],position[2]);
//    fwrite(buf_temp,1,strlen(buf_temp),fpned);

    return 0;
}
