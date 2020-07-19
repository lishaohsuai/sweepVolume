#include "tool.h"
#include <QFileDialog>
#include <fstream>
#include <QGLWidget>
#include <gl/glu.h>



/**************************************************
@brief   : ���캯�� �򵥵ĳ�ʼ��������ֵ
@author  : lee
@input   ��parent һ��ΪNULL
@output  ��none
@time    : none
**************************************************/
Tool::Tool(QWidget *parent) :
	Strategy(), QGLWidget(parent) {
	flag = -1;// û���κε�����¼�
	rotationX = 0;
	rotationY = 0;
	rotationZ = 0;
};



/**************************************************
@brief   : ���������ͷ������Դ
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/
Tool::~Tool() {}




/**************************************************
@brief   : �򵥵Ľ����ɵĵ���뵽������
@author  : lee
@input   ��str  �ļ�·��
@output  ��none
@time    : none
**************************************************/
void Tool::genPoints(std::string str) {
	jsonReader(str);
	std::cout << "[DEBUG]genPoint" << std::endl;
}


/**************************************************
@brief   : ��xoyƽ��ƽ�е��м��ı任
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/
void Tool::flipWithMidLine(const std::vector<Point3d> &oldPoint, std::vector<Point3d> &newPoint) {
	Point3d sum(0, 0, 0);
	newPoint.resize(oldPoint.size());
	for (int i = 0; i < oldPoint.size(); i++) {
		
	}
	for (int i = 0; i < oldPoint.size(); i++) {
		sum.x = sum.x + oldPoint[i].x;
		sum.y = sum.y + oldPoint[i].y;
		sum.z = sum.z + oldPoint[i].z;
	}
	Point3d Avg(0, 0, 0);
	Avg.x = (double)sum.x / oldPoint.size();
	Avg.y = (double)sum.y / oldPoint.size();
	Avg.z = (double)sum.z / oldPoint.size();
	for (int i = 0; i < oldPoint.size(); i++) {
		newPoint[i].x = oldPoint[i].x;
		newPoint[i].y = oldPoint[i].y;
		newPoint[i].z = Avg.z - (oldPoint[i].z - Avg.z);
	}
	for (int i = 0; i < oldPoint.size(); i++) {
		std::cout << "[" << newPoint[i].x << "," << newPoint[i].y << "," << newPoint[i].z << "],";
	}
}

/**************************************************
@brief   : ���ļ��ж�ȡ��Ӧ�Ĳ���
@author  : lee
@input   ���ļ���(����·��)/��� �������
@output  ��none
@time    : none
**************************************************/
void Tool::jsonReader(std::string fileName) {
	if (fileName == "") {
		QString QfileName = QFileDialog::getOpenFileName(NULL,
			QObject::tr("Open json file"),
			QObject::tr("./config"),
			QObject::tr("mesh (*.json);;"
				"All Files (*)"));
		fileName = QfileName.toStdString();
	}
	Json::Value root;
	Json::Reader reader;
	std::ifstream ifs(fileName.c_str());//open file example.json

	if (!reader.parse(ifs, root)) {
		// fail to parse
		std::cout << "fail to parse json \n";
	}
	else {
		// success
		std::cout << "[DEBUG] config file have found!!\n";
		std::cout << "name: " << root["name"].asString() << std::endl;
		Json::Value points = root["curveCtrlPoints"];
		std::vector<Point3d> ctrlPoints;
		for (int i = 0; i < points.size(); i++) {
			Point3d p(0, 0, 0);
			p.x = (points[i][0].asDouble());
			p.y = (points[i][1].asDouble());
			p.z = (points[i][2].asDouble());
			ctrlPoints.push_back(p);
		}
		std::vector<Point3d> newPoints;
		flipWithMidLine(ctrlPoints, newPoints);
	}
}


/**************************************************
@brief   : ��ʼ��opengl����
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/
void Tool::initializeGL() {
	glShadeModel(GL_SMOOTH);
	glClearColor(0.1, 0.1, 0.4, 1.0);
	glClearDepth(1.0);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}


/**************************************************
@brief   : �ı䴰�ڵĴ�С
@author  : lee
@input   ��width ���
		   height �߶�
@output  ��none
@time    : none
**************************************************/
void Tool::resizeGL(int width, int height)
{
	// ��ֹheightΪ0
	if (height == 0) {
		height = 1;
	}
	// ���õ�ǰ���ӿ�
	glViewport(0, 0, (GLint)width, (GLint)height);
	// ѡ��ͶӰ����
	glMatrixMode(GL_PROJECTION);
	// ���ù۲����/ͶӰ���� �����ôκ�����ʵ�ʽ���ǰ���Ƶ�����Ļ����
	glLoadIdentity();
	// ����͸��ͶӰ����,��Ҫ<GL/glu.h>ͷ�ļ�
	gluPerspective(45.0, (GLfloat)width / (GLfloat)height, 0.1, 100.0);//�Ƕȣ�����ȣ�Զ��
	// ѡ��ģ�͹۲����
	glMatrixMode(GL_MODELVIEW);
	// ���ù۲����/ͶӰ����
	glLoadIdentity();
}


/**************************************************
@brief   : �Զ��ͱ��� ���ø�������ͼ��
@author  : none
@input   ��none
@output  ��none
@time    : none
**************************************************/
void Tool::paintGL() {
	//genPoints("normal");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// �����Ļ����Ȼ���
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//ƽ��

	glRotated(rotationX, 1.0, 0.0, 0.0);
	glRotated(rotationY, 0.0, 1.0, 0.0);
	glRotated(rotationZ, 0.0, 0.0, 1.0);//��ת

	//// �����Ƶ�
	//glPointSize(8.0);
	//glBegin(GL_POINTS);
	//glColor3f(1.0, 0.0, 0.0);
	//for (int i = 0; i < nurbsCvCfg.ctrlPoints.size(); i++) {
	//	glVertex3d(nurbsCvCfg.ctrlPoints[i].x, nurbsCvCfg.ctrlPoints[i].y, nurbsCvCfg.ctrlPoints[i].z);
	//}
	//glEnd();

	//// �������ƶ����
	//glBegin(GL_LINE_STRIP);
	//glColor3f(0.0, 1.0, 0.0);
	//for (int i = 0; i < nurbsCvCfg.ctrlPoints.size(); i++) {
	//	glVertex3d(nurbsCvCfg.ctrlPoints[i].x, nurbsCvCfg.ctrlPoints[i].y, nurbsCvCfg.ctrlPoints[i].z);
	//}
	//glEnd();

	//// ��������
	//glBegin(GL_LINE_STRIP);
	//glColor3f(0.0, 0.0, 1.0);
	//for (int i = 0; i < nurbsCvCfg.curvaturePoints.size(); i++) {
	//	glVertex3f(nurbsCvCfg.curvaturePoints[i].x, nurbsCvCfg.curvaturePoints[i].y, nurbsCvCfg.curvaturePoints[i].z);
	//}
	//glEnd();
}



/**************************************************
@brief   : ���������¼�
@author  : lee
@input   ��e  �¼�
@output  ��none
@time    : none
**************************************************/
void Tool::mousePressEvent(QMouseEvent *e) {
	lastPos = e->pos();
	flag = 1;
}


/**************************************************
@brief   : none
@author  : none
@input   ��none
@output  ��none
@time    : none
**************************************************/
void Tool::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : none
@author  : none
@input   ��none
@output  ��none
@time    : none
**************************************************/
void Tool::mouseMoveEvent(QMouseEvent *e) {
	if (flag) {
		GLdouble dx = GLdouble(e->x() - lastPos.x()) / width();//QWidght �� ���
		GLdouble dy = GLdouble(e->y() - lastPos.y()) / height();
		if (e->buttons() & Qt::LeftButton) {
			rotationX -= 180 * dy;
			rotationY -= 180 * dx;
			update();
		}
		else if (e->buttons() & Qt::RightButton) {
			rotationX -= 180 * dy;
			rotationZ -= 180 * dx;
			update();
		}
		lastPos = e->pos();
	}
}