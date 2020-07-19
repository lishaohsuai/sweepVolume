#include "tool.h"
#include <QFileDialog>
#include <fstream>
#include <QGLWidget>
#include <gl/glu.h>



/**************************************************
@brief   : 构造函数 简单的初始化常量赋值
@author  : lee
@input   ：parent 一般为NULL
@output  ：none
@time    : none
**************************************************/
Tool::Tool(QWidget *parent) :
	Strategy(), QGLWidget(parent) {
	flag = -1;// 没有任何的鼠标事件
	rotationX = 0;
	rotationY = 0;
	rotationZ = 0;
};



/**************************************************
@brief   : 析构函数释放相关资源
@author  : lee
@input   ：none
@output  ：none
@time    : none
**************************************************/
Tool::~Tool() {}




/**************************************************
@brief   : 简单的讲生成的点加入到曲线上
@author  : lee
@input   ：str  文件路径
@output  ：none
@time    : none
**************************************************/
void Tool::genPoints(std::string str) {
	jsonReader(str);
	std::cout << "[DEBUG]genPoint" << std::endl;
}


/**************************************************
@brief   : 和xoy平面平行的中间点的变换
@author  : lee
@input   ：none
@output  ：none
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
@brief   : 从文件中读取相应的参数
@author  : lee
@input   ：文件名(完整路径)/或空 方便调试
@output  ：none
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
@brief   : 初始化opengl环境
@author  : lee
@input   ：none
@output  ：none
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
@brief   : 改变窗口的大小
@author  : lee
@input   ：width 宽度
		   height 高度
@output  ：none
@time    : none
**************************************************/
void Tool::resizeGL(int width, int height)
{
	// 防止height为0
	if (height == 0) {
		height = 1;
	}
	// 重置当前的视口
	glViewport(0, 0, (GLint)width, (GLint)height);
	// 选择投影矩阵
	glMatrixMode(GL_PROJECTION);
	// 重置观察矩阵/投影矩阵 当调用次函数，实际将当前点移到了屏幕中心
	glLoadIdentity();
	// 建立透视投影矩阵,需要<GL/glu.h>头文件
	gluPerspective(45.0, (GLfloat)width / (GLfloat)height, 0.1, 100.0);//角度，长宽比，远近
	// 选择模型观察矩阵
	glMatrixMode(GL_MODELVIEW);
	// 重置观察矩阵/投影矩阵
	glLoadIdentity();
}


/**************************************************
@brief   : 自动和被动 调用更新整个图形
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/
void Tool::paintGL() {
	//genPoints("normal");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// 清除屏幕和深度缓存
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//平移

	glRotated(rotationX, 1.0, 0.0, 0.0);
	glRotated(rotationY, 0.0, 1.0, 0.0);
	glRotated(rotationZ, 0.0, 0.0, 1.0);//旋转

	//// 画控制点
	//glPointSize(8.0);
	//glBegin(GL_POINTS);
	//glColor3f(1.0, 0.0, 0.0);
	//for (int i = 0; i < nurbsCvCfg.ctrlPoints.size(); i++) {
	//	glVertex3d(nurbsCvCfg.ctrlPoints[i].x, nurbsCvCfg.ctrlPoints[i].y, nurbsCvCfg.ctrlPoints[i].z);
	//}
	//glEnd();

	//// 画出控制多边形
	//glBegin(GL_LINE_STRIP);
	//glColor3f(0.0, 1.0, 0.0);
	//for (int i = 0; i < nurbsCvCfg.ctrlPoints.size(); i++) {
	//	glVertex3d(nurbsCvCfg.ctrlPoints[i].x, nurbsCvCfg.ctrlPoints[i].y, nurbsCvCfg.ctrlPoints[i].z);
	//}
	//glEnd();

	//// 画出曲线
	//glBegin(GL_LINE_STRIP);
	//glColor3f(0.0, 0.0, 1.0);
	//for (int i = 0; i < nurbsCvCfg.curvaturePoints.size(); i++) {
	//	glVertex3f(nurbsCvCfg.curvaturePoints[i].x, nurbsCvCfg.curvaturePoints[i].y, nurbsCvCfg.curvaturePoints[i].z);
	//}
	//glEnd();
}



/**************************************************
@brief   : 按键按下事件
@author  : lee
@input   ：e  事件
@output  ：none
@time    : none
**************************************************/
void Tool::mousePressEvent(QMouseEvent *e) {
	lastPos = e->pos();
	flag = 1;
}


/**************************************************
@brief   : none
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/
void Tool::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : none
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/
void Tool::mouseMoveEvent(QMouseEvent *e) {
	if (flag) {
		GLdouble dx = GLdouble(e->x() - lastPos.x()) / width();//QWidght 的 宽度
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