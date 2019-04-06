/////////////////////////////////////////////////////////////////////////
//@Copyright(C) Pyrokine
//All rights reserved
//博客 http://www.cnblogs.com/Pyrokine/
//Github https://github.com/Pyrokine
/////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <rplidar.h>
#include <math.h>
#include <glut.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#endif

//声明命名空间
using namespace rp::standalone::rplidar;
//声明结构图
typedef struct _struct_data_polar
{
	double the;
	double dist;
} struct_data_polar;
typedef struct _stuct_data_decare
{
	double loc[360][2];
	int size;
} struct_data_decare;
//声明全局变量
RPlidarDriver *drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
u_result op_result;
struct_data_polar data_polar[360];
struct_data_decare data_filter[360];
struct_data_decare data_block[360];
GLdouble DOUBLE_PI = 2.0f * 3.1415926f;
GLdouble PI = 3.1415926;
rplidar_response_measurement_node_t nodes[360];
rplidar_response_measurement_node_t nodes_second[360];
size_t count = _countof(nodes);
int cnt_data_polar = 0, cnt_data_decare = 0, cnt_data_filter = 0, cnt_data_block = 0;
int zoom_scene = 300, x_delta = 0, y_delta = 0, x_fisrt_click = 0, y_first_click = 0;
int x_scene_moved = 0, y_scene_moved = 0;
float zoom_mouse = 0;
int is_ctrl_press = 0, is_left_press = 0;
double data_decare[360][2];
//声明函数
void RadarInitial(void);
void GetLocation(void);
void PolarToDecare();
void FilterRawData();
void Fragment(int num_data_filter, int size_data_filter);
void RenderScene(void);
void ChangeSize(int w, int h);
void DrawLine(double x1, double y1, double x2, double y2);
void DrawSolidCircle(float x, float y, float radius);
void DrawWiredCircle(float x, float y, float radius);
void ProcessNormalKeys(unsigned char key, int x, int y);
void ProcessSpecialKeys(int key, int x, int y);
void ProcessKeysUP(unsigned char key, int x, int y);
void ProcessMouse(int button, int state, int x, int y);
void ProcessMotion(int x, int y);
void StartScan(void);
void StopScan(void);
void MyGlutInitial(int argc, char **argv);
//主循环
int main(int argc, char **argv)
{
	RadarInitial();
	StartScan();
	MyGlutInitial(argc, argv);
}
//初始化函数
void RadarInitial(void)
{
	//设备所在的端口号和进行通讯所用的比特率
	const char *opt_com_path = "\\\\.\\com18";
	_u32 opt_com_baudrate = 115200;
	//声明变量
	rplidar_response_device_info_t devinfo;
	rplidar_response_device_health_t healthinfo;
	//检查drv有效性
	if (!drv)
	{
		printf("内存不足，按任意键退出\n");
		system("pause");
		exit(0);
	}
	//用获得的端口号和比特率与雷达建立连接
	if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate)))
	{
		printf("无法与相应端口建立连接，按任意键退出\n");
		system("pause");
		exit(0);
	}
	//获取设备硬件信息
	op_result = drv->getDeviceInfo(devinfo);
	//检验能否获得数据
	if (IS_FAIL(op_result))
	{
		printf("无法获得设备的硬件信息，按任意键退出\n");
		system("pause");
		exit(0);
	}
	//输出设备硬件信息
	printf("设备序列号：");
	for (int pos = 0; pos < 16; ++pos)
	{
		printf("%02X", devinfo.serialnum[pos]);
	}
	printf("\n");
	printf("固件版本：%d.%02d\n", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
	printf("硬件版本：%d\n", (int)devinfo.hardware_version);
	//获取设备健康信息
	op_result = drv->getHealth(healthinfo);
	//检验能否获得数据
	if (IS_FAIL(op_result))
	{
		printf("无法获得设备的健康信息，错误代码是%x，按任意键退出\n", op_result);
		system("pause");
		exit(0);
	}
	//输出设备健康信息
	printf("设备健康值：%d\n", healthinfo.status);
	//对设备不健康的情况进行处理
	if (healthinfo.status == RPLIDAR_STATUS_ERROR)
	{
		printf("设备不健康，请重启设备后重试，按任意键退出\n");
		system("pause");
		exit(0);
	}
}
//获取雷达数据
void GetLocation(void)
{
	//初始化变量
	int cnt_temp = 0;
	cnt_data_polar = 0;
	drv->grabScanData(nodes, count);
	for (int pos = 0; pos < (int)count; ++pos)
	{
		if (nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT != 0)
		{
			data_polar[cnt_data_polar].the = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
			data_polar[cnt_data_polar].dist = nodes[pos].distance_q2 / 4.0f;
			cnt_data_polar++;
		}
	}
}
//将极坐标转化为笛卡尔坐标并进行范围阈值设定
void PolarToDecare()
{
	int cnt_temp = 0;
	cnt_data_decare = 0;
	for (cnt_temp = 0; cnt_temp < cnt_data_polar; cnt_temp++)
	{
		if (data_polar[cnt_temp].dist < 2000)
		{
			data_decare[cnt_data_decare][0] = data_polar[cnt_temp].dist * sin(data_polar[cnt_temp].the / 180 * PI);
			data_decare[cnt_data_decare][1] = data_polar[cnt_temp].dist * cos(data_polar[cnt_temp].the / 180 * PI);
			//DrawSolidCircle(data_decare[cnt_data_decare][0] / zoom_scene + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, data_decare[cnt_data_decare][1] / zoom_scene - 1.0f * (y_delta + y_scene_moved) / zoom_mouse, 0.03);
			cnt_data_decare++;
		}
	}
}
//将数据初步分块
void FilterRawData()
{
	int cnt_temp = 0, cnt_temp2 = 0;
	cnt_data_filter = 0;
	cnt_data_block = 0;
	for (cnt_temp = 0; cnt_temp < cnt_data_decare - 1; cnt_temp++)
	{
		data_filter[cnt_data_filter].loc[cnt_temp2][0] = data_decare[cnt_temp][0];
		data_filter[cnt_data_filter].loc[cnt_temp2][1] = data_decare[cnt_temp][1];
		//DrawSolidCircle(data_decare[cnt_temp][0] / zoom_scene + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, data_decare[cnt_temp][1] / zoom_scene - 1.0f * (y_delta + y_scene_moved) / zoom_mouse, 0.03);
		//DrawSolidCircle(data_filter[cnt_data_filter].loc[cnt_temp2][0] / zoom_scene + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, data_filter[cnt_data_filter].loc[cnt_temp2][1] / zoom_scene - 1.0f * (y_delta + y_scene_moved) / zoom_mouse, 0.03);
		cnt_temp2++;
		double dist = sqrt(pow(data_decare[cnt_temp][0] - data_decare[cnt_temp + 1][0], 2) + pow(data_decare[cnt_temp][1] - data_decare[cnt_temp + 1][1], 2));
		if (dist > 100)
		{
			Fragment(cnt_data_filter, cnt_temp2);
			cnt_data_filter++;
			cnt_temp2 = 0;
		}
	}
}
//将数据分块
void Fragment(int num_data_filter, int size_data_filter)
{
	//printf("%d ", num_data_filter);
	if (fabs(data_filter[num_data_filter].loc[0][0]) < 1)
		return;
	else if (size_data_filter <= 3)
	{
		if (size_data_filter == 3)
		{
			int cnt_temp = 0;
			for (cnt_temp = 0; cnt_temp < size_data_filter; cnt_temp++)
			{
				data_block[cnt_data_block].loc[cnt_temp][0] = data_filter[num_data_filter].loc[cnt_temp][0];
				data_block[cnt_data_block].loc[cnt_temp][1] = data_filter[num_data_filter].loc[cnt_temp][1];
			}
			data_block[cnt_data_block].size = size_data_filter;
			cnt_data_block++;
			return;
		}
		else
			return;
	}
	//y = kx + d
	double k = (data_filter[num_data_filter].loc[size_data_filter - 1][1] - data_filter[num_data_filter].loc[0][1]) / (data_filter[num_data_filter].loc[size_data_filter - 1][0] - data_filter[num_data_filter].loc[0][0]);
	double d = data_filter[num_data_filter].loc[size_data_filter - 1][1] - k * data_filter[num_data_filter].loc[size_data_filter - 1][0];
	double dist_max = 0;
	int cnt_max = 0, cnt_temp = 0;
	for (cnt_temp = 1; cnt_temp < size_data_filter - 1; cnt_temp++)
	{
		double dist = fabs(k * data_filter[num_data_filter].loc[cnt_temp][0] - data_filter[num_data_filter].loc[cnt_temp][1] + d) / sqrt(pow(k, 2) + 1);
		if (dist > dist_max)
		{
			dist_max = dist;
			cnt_max = cnt_temp;
		}
	}
	double dist = sqrt(pow(data_filter[num_data_filter].loc[size_data_filter - 1][0] - data_filter[num_data_filter].loc[0][0], 2) + pow(data_filter[num_data_filter].loc[size_data_filter - 1][1] - data_filter[num_data_filter].loc[0][1], 2));
	if (dist_max > dist / 10)
	{
		cnt_data_filter++;
		for (cnt_temp = cnt_max; cnt_temp < size_data_filter; cnt_temp++)
		{
			data_filter[cnt_data_filter].loc[cnt_temp - cnt_max][0] = data_filter[num_data_filter].loc[cnt_temp][0];
			data_filter[cnt_data_filter].loc[cnt_temp - cnt_max][1] = data_filter[num_data_filter].loc[cnt_temp][1];
		}
		Fragment(cnt_data_filter, size_data_filter - cnt_max);
		Fragment(num_data_filter, cnt_max + 1);
	}
	else
	{
		int cnt_temp = 0;
		for (cnt_temp = 0; cnt_temp < size_data_filter; cnt_temp++)
		{
			data_block[cnt_data_block].loc[cnt_temp][0] = data_filter[num_data_filter].loc[cnt_temp][0];
			data_block[cnt_data_block].loc[cnt_temp][1] = data_filter[num_data_filter].loc[cnt_temp][1];
		}
		data_block[cnt_data_block].size = size_data_filter;
		cnt_data_block++;
		
	}
}
//显示函数
void RenderScene(void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	GetLocation();
	PolarToDecare();
	FilterRawData();
	printf("\n");
	//printf("%d\n", cnt_data_block);
	int cnt_temp = 0;
	glPushMatrix();
	//绘制网格辅助线
	//glBegin(GL_LINES);
	for (cnt_temp = 0; cnt_temp < cnt_data_block; cnt_temp++)
	{
		DrawLine(data_block[cnt_temp].loc[0][0], data_block[cnt_temp].loc[0][1], data_block[cnt_temp].loc[data_block[cnt_temp].size - 1][0], data_block[cnt_temp].loc[data_block[cnt_temp].size - 1][1]);
	}
	glVertex2d(-10000.0f / zoom_scene + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, 0 - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glVertex2d(10000.0f / zoom_scene + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, 0 - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glVertex2d(0 + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, -10000.0f / zoom_scene - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glVertex2d(0 + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, 10000.0f / zoom_scene - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glVertex2d(-10000.0f / zoom_scene * cos(PI * 30 / 180) + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, -10000.0f / zoom_scene * sin(PI * 30 / 180) - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glVertex2d(10000.0f / zoom_scene * cos(PI * 30 / 180) + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, 10000.0f / zoom_scene * sin(PI * 30 / 180) - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glVertex2d(-10000.0f / zoom_scene * cos(PI * 60 / 180) + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, -10000.0f / zoom_scene * sin(PI * 60 / 180) - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glVertex2d(10000.0f / zoom_scene * cos(PI * 60 / 180) + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, 10000.0f / zoom_scene * sin(PI * 60 / 180) - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glVertex2d(-10000.0f / zoom_scene * cos(PI * 120 / 180) + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, -10000.0f / zoom_scene * sin(PI * 120 / 180) - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glVertex2d(10000.0f / zoom_scene * cos(PI * 120 / 180) + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, 10000.0f / zoom_scene * sin(PI * 120 / 180) - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glVertex2d(-10000.0f / zoom_scene * cos(PI * 150 / 180) + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, -10000.0f / zoom_scene * sin(PI * 150 / 180) - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glVertex2d(10000.0f / zoom_scene * cos(PI * 150 / 180) + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, 10000.0f / zoom_scene * sin(PI * 150 / 180) - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	//glEnd();
	for (cnt_temp = 1000; cnt_temp <= 10000; cnt_temp = cnt_temp + 1000)
	{
		DrawWiredCircle(1.0f * (x_delta + x_scene_moved) / zoom_mouse, -1.0f * (y_delta + y_scene_moved) / zoom_mouse, 1.0f * cnt_temp / zoom_scene);
	}
	//绘制雷达点
	//for (cnt_temp = 0; cnt_temp < cnt_data_polar; cnt_temp++)
	//{
	//	DrawSolidCircle(data_polar[cnt_temp].dist * sin(data_polar[cnt_temp].the / 180 * PI) / zoom_scene + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, data_polar[cnt_temp].dist * cos(data_polar[cnt_temp].the / 180 * PI) / zoom_scene - 1.0f * (y_delta + y_scene_moved) / zoom_mouse, 0.05);
	//}
	if (is_left_press == 2)
	{
		is_left_press = 0;
		x_scene_moved += x_delta;
		y_scene_moved += y_delta;
		x_delta = y_delta = 0;
	}
	glPopMatrix();
	glutSwapBuffers();
	glutPostRedisplay();
}
//图像的映射，使得画面缩放时图像不会形变
void ChangeSize(int w, int h)
{
	if (h == 0)
		h = 1;
	float ratio = 1.0 * w / h;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);
	gluPerspective(70, ratio, 0.1, 3000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0f, 1.0f, 0.0f);
	int window_width, window_height;
	window_width = glutGet(GLUT_WINDOW_WIDTH);
	window_height = glutGet(GLUT_WINDOW_HEIGHT);
	if (window_width <= window_height)
		zoom_mouse = 71.5f * window_width / 1000;
	else if (window_width > window_height)
		zoom_mouse = 71.5f * window_height / 1000;
}
//绘制线段
void DrawLine(double x1, double y1, double x2, double y2)
{
	glBegin(GL_LINES);
	glVertex2d(x1 / zoom_scene + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, y1 / zoom_scene - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glVertex2d(x2 / zoom_scene + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, y2 / zoom_scene - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glEnd();
}
//用扇形绘制实心圆
void DrawSolidCircle(float x, float y, float radius)
{
	int count;
	int sections = 5;

	glBegin(GL_TRIANGLE_FAN);
	glVertex2d(x, y);
	for (count = 0; count <= sections; count++)
	{
		glVertex2d(x + radius * cos(count * DOUBLE_PI / sections), y + radius * sin(count * DOUBLE_PI / sections));
	}
	glEnd();
}
//用多边形绘制空心圆
void DrawWiredCircle(float x, float y, float radius)
{
	int count;
	int sections = 200;

	glBegin(GL_LINE_STRIP);
	for (count = 0; count <= sections; count++)
	{
		glVertex2d(x + radius * cos(count * DOUBLE_PI / sections), y + radius * sin(count * DOUBLE_PI / sections));
	}
	glEnd();
}
//处理普通按键事件
void ProcessNormalKeys(unsigned char key, int x, int y)
{
	if (key == 27)
	{
		StopScan();
		exit(0);
	}
}
//处理特殊按键事件
void ProcessSpecialKeys(int key, int x, int y)
{

}
//处理按键抬起事件
void ProcessKeysUP(unsigned char key, int x, int y)
{

}
//处理鼠标事件
void ProcessMouse(int button, int state, int x, int y)
{
	if (button == GLUT_WHEEL_UP && state == GLUT_UP)
	{
		if (glutGetModifiers() == GLUT_ACTIVE_CTRL)
			zoom_scene -= 50;
		else
			zoom_scene -= 10;

		if (zoom_scene < 10)
			zoom_scene = 10;

		glutPostRedisplay();
	}
	else if (button == GLUT_WHEEL_DOWN && state == GLUT_UP)
	{
		if (glutGetModifiers() == GLUT_ACTIVE_CTRL)
			zoom_scene += 50;
		else
			zoom_scene += 10;

		if (zoom_scene > 3000)
			zoom_scene = 3000;

		glutPostRedisplay();
	}
	else if (button == GLUT_LEFT_BUTTON)
	{
		if (is_left_press == 0)
		{
			x_fisrt_click = x;
			y_first_click = y;
			is_left_press = 1;
		}
		else if (is_left_press == 1)
		{
			is_left_press = 2;
			glutPostRedisplay();
		}
	}
}
//鼠标按下后返回鼠标位置
void ProcessMotion(int x, int y)
{
	x_delta = x - x_fisrt_click;
	y_delta = y - y_first_click;
	if (is_left_press == 1)
		glutPostRedisplay();
}
//启动电机并扫描
void StartScan(void)
{
	drv->startMotor();
	drv->startScan();
	//获取扫描到的数据并储存到nodes
	op_result = drv->grabScanData(nodes, count);
	//检验能否获得数据
	if (IS_FAIL(op_result))
	{
		printf("无法获取雷达扫描结果,按任意键退出\n");
		system("pause");
		exit(0);
	}
}
//停止电机并停止扫描
void StopScan(void)
{
	drv->stop();
	drv->stopMotor();
	RPlidarDriver::DisposeDriver(drv);
}
//初始化GLUT
void MyGlutInitial(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(600, 600);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("uslam");
	glutDisplayFunc(RenderScene);
	glutReshapeFunc(ChangeSize);
	glutKeyboardFunc(ProcessNormalKeys);
	glutSpecialFunc(ProcessSpecialKeys);
	glutKeyboardUpFunc(ProcessKeysUP);
	glutMouseFunc(ProcessMouse);
	glutMotionFunc(ProcessMotion);
	glutIdleFunc(RenderScene);
	//glutFullScreen();
	glutMainLoop();
}