/////////////////////////////////////////////////////////////////////////
//@Copyright(C) Pyrokine
//All rights reserved
//���� http://www.cnblogs.com/Pyrokine/
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

//���������ռ�
using namespace rp::standalone::rplidar;
//�����ṹͼ
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
//����ȫ�ֱ���
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
//��������
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
//��ѭ��
int main(int argc, char **argv)
{
	RadarInitial();
	StartScan();
	MyGlutInitial(argc, argv);
}
//��ʼ������
void RadarInitial(void)
{
	//�豸���ڵĶ˿ںźͽ���ͨѶ���õı�����
	const char *opt_com_path = "\\\\.\\com18";
	_u32 opt_com_baudrate = 115200;
	//��������
	rplidar_response_device_info_t devinfo;
	rplidar_response_device_health_t healthinfo;
	//���drv��Ч��
	if (!drv)
	{
		printf("�ڴ治�㣬��������˳�\n");
		system("pause");
		exit(0);
	}
	//�û�õĶ˿ںźͱ��������״ｨ������
	if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate)))
	{
		printf("�޷�����Ӧ�˿ڽ������ӣ���������˳�\n");
		system("pause");
		exit(0);
	}
	//��ȡ�豸Ӳ����Ϣ
	op_result = drv->getDeviceInfo(devinfo);
	//�����ܷ�������
	if (IS_FAIL(op_result))
	{
		printf("�޷�����豸��Ӳ����Ϣ����������˳�\n");
		system("pause");
		exit(0);
	}
	//����豸Ӳ����Ϣ
	printf("�豸���кţ�");
	for (int pos = 0; pos < 16; ++pos)
	{
		printf("%02X", devinfo.serialnum[pos]);
	}
	printf("\n");
	printf("�̼��汾��%d.%02d\n", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
	printf("Ӳ���汾��%d\n", (int)devinfo.hardware_version);
	//��ȡ�豸������Ϣ
	op_result = drv->getHealth(healthinfo);
	//�����ܷ�������
	if (IS_FAIL(op_result))
	{
		printf("�޷�����豸�Ľ�����Ϣ�����������%x����������˳�\n", op_result);
		system("pause");
		exit(0);
	}
	//����豸������Ϣ
	printf("�豸����ֵ��%d\n", healthinfo.status);
	//���豸��������������д���
	if (healthinfo.status == RPLIDAR_STATUS_ERROR)
	{
		printf("�豸���������������豸�����ԣ���������˳�\n");
		system("pause");
		exit(0);
	}
}
//��ȡ�״�����
void GetLocation(void)
{
	//��ʼ������
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
//��������ת��Ϊ�ѿ������겢���з�Χ��ֵ�趨
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
//�����ݳ����ֿ�
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
//�����ݷֿ�
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
//��ʾ����
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
	//������������
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
	//�����״��
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
//ͼ���ӳ�䣬ʹ�û�������ʱͼ�񲻻��α�
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
//�����߶�
void DrawLine(double x1, double y1, double x2, double y2)
{
	glBegin(GL_LINES);
	glVertex2d(x1 / zoom_scene + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, y1 / zoom_scene - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glVertex2d(x2 / zoom_scene + 1.0f * (x_delta + x_scene_moved) / zoom_mouse, y2 / zoom_scene - 1.0f * (y_delta + y_scene_moved) / zoom_mouse);
	glEnd();
}
//�����λ���ʵ��Բ
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
//�ö���λ��ƿ���Բ
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
//������ͨ�����¼�
void ProcessNormalKeys(unsigned char key, int x, int y)
{
	if (key == 27)
	{
		StopScan();
		exit(0);
	}
}
//�������ⰴ���¼�
void ProcessSpecialKeys(int key, int x, int y)
{

}
//������̧���¼�
void ProcessKeysUP(unsigned char key, int x, int y)
{

}
//��������¼�
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
//��갴�º󷵻����λ��
void ProcessMotion(int x, int y)
{
	x_delta = x - x_fisrt_click;
	y_delta = y - y_first_click;
	if (is_left_press == 1)
		glutPostRedisplay();
}
//���������ɨ��
void StartScan(void)
{
	drv->startMotor();
	drv->startScan();
	//��ȡɨ�赽�����ݲ����浽nodes
	op_result = drv->grabScanData(nodes, count);
	//�����ܷ�������
	if (IS_FAIL(op_result))
	{
		printf("�޷���ȡ�״�ɨ����,��������˳�\n");
		system("pause");
		exit(0);
	}
}
//ֹͣ�����ֹͣɨ��
void StopScan(void)
{
	drv->stop();
	drv->stopMotor();
	RPlidarDriver::DisposeDriver(drv);
}
//��ʼ��GLUT
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