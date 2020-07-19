#include "bsplineCurve3d.h"
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
BsplineCurve3d::BsplineCurve3d(QWidget *parent) :
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
BsplineCurve3d::~BsplineCurve3d() {}

/**************************************************
@brief   : 找到 u 所属于的区间序列
@author  : lee
@input   : numOfCtlPoint 控制点的个数
		   order 基函数的阶
		   u 递增的一般从0到1上的点
		   U 节点向量
@output  : 节点区间序列
@time    : none
**************************************************/
int BsplineCurve3d::findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U) {
	if (u == U[numOfCtlPoint + 1]) return numOfCtlPoint;
	int low = order;
	int high = numOfCtlPoint + 1;
	int mid = (low + high) / 2;
	while (u < U[mid] || u >= U[mid + 1]) {
		if (u < U[mid]) high = mid;
		else low = mid;
		mid = (low + high) / 2;
	}
	return mid;
}

/**************************************************
@brief   : 计算非空的基函数
		   第一次，p=0时，只有N[0]=1.0,即N0,0=1.0;p=1时，求N[0],N[1],即N0,1和N1,1;p=2时，求N[0],N[1],N[2],即N0,2  N1,2和N2,2
@author  : lee
@input   ：i 节点向量的第i个序列
		   u 生成曲线上的点
		   order 阶次
		   U 节点向量
@output  ：base 从第0阶到后面的阶辐射的三角形所对应的阶函数的值，一整套
@time    : none
**************************************************/
void BsplineCurve3d::basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base) {
	base.resize(order + 1);
	base[0] = 1.0;
	std::vector<double> left, right;
	left.resize(order + 1);
	right.resize(order + 1);
	for (int j = 1; j <= order; j++) {
		left[j] = u - U[i + 1 - j];
		right[j] = U[i + j] - u;
		double saved = 0.0;
		for (int r = 0; r < j; r++) {
			double temp = base[r] / (right[r + 1] + left[j - r]);
			base[r] = saved + right[r + 1] * temp;
			saved = left[j - r] * temp;
		}
		base[j] = saved;
	}
}


/**************************************************
@brief   : 生成曲线上的点
		   关键公式  m = n + p + 1
@author  : lee
@input   : n+1 控制点的个数
		   p 阶次
		   m+1 节点的个数
		   P  控制点数组
		   u 从小到大，从而绘制出整个曲线
		   C 生成的点的坐标
@output  ：none
@time    : none
**************************************************/
void BsplineCurve3d::curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u, Point3d &C) {
	int span = findSpan(n, p, u, U);
	std::vector<double> base;
	basisFuns(span, u, p, U, base);
	double x, y, z;
	x = y = z = 0;
	for (int i = 0; i <= p; i++) {
		x = x +  P[span - p + i].x * base[i];
		y = y + base[i] * P[span - p + i].y;
		z = z + base[i] * P[span - p + i].z;
	}
	Point3d temp;
	temp.x = x;
	temp.y = y;
	temp.z = z;
	C = temp;
}

/**************************************************
@brief   : 简单的讲生成的点加入到曲线上
@author  : lee
@input   ：str  文件路径
@output  ：none
@time    : none
**************************************************/
void BsplineCurve3d::genPoints(std::string str) {
	jsonReader(str);
	std::cout << "[DEBUG]genPoint" << std::endl;
	bs3dConfig.curvaturePoints.clear();
	for (int i = 0; i <= bs3dConfig.curvaturePointsNumber; i++) {
		double u = i * 1.0 * (bs3dConfig.U_vector[bs3dConfig.U_vector.size() - 1] - bs3dConfig.U_vector[0]) / bs3dConfig.curvaturePointsNumber;//均匀分成多少个点再所有的节点上
		Point3d C;
		curvePoint(bs3dConfig.ctrlPoints.size() - 1, bs3dConfig.X_Power, bs3dConfig.U_vector, bs3dConfig.ctrlPoints, u, C);
		bs3dConfig.curvaturePoints.push_back(C);
	}
}



/**************************************************
@brief   : 从文件中读取相应的参数
@author  : lee
@input   ：文件名(完整路径)/或空 方便调试
@output  ：none
@time    : none
**************************************************/
void BsplineCurve3d::jsonReader(std::string fileName) {
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
		bs3dConfig.name = root["name"].asString();
		bs3dConfig.X_Power = root["X_Power"].asInt();
		bs3dConfig.curvaturePointsNumber = root["curvePointNum"].asInt();
		Json::Value points = root["ctrlPoint"];
		for (int i = 0; i < points.size(); i++) {
			Point3d p(0, 0, 0);
			p.x = (points[i][0].asDouble());
			p.y = (points[i][1].asDouble());
			p.z = (points[i][2].asDouble());
			bs3dConfig.ctrlPoints.push_back(p);
		}
		Json::Value vectors = root["U_vector"];
		for (int i = 0; i < vectors.size(); i++) {
			bs3dConfig.U_vector.push_back(vectors[i].asDouble());
		}
	}
}


/**************************************************
@brief   : 初始化opengl环境
@author  : lee
@input   ：none
@output  ：none
@time    : none
**************************************************/
void BsplineCurve3d::initializeGL() {
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
void BsplineCurve3d::resizeGL(int width, int height)
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
void BsplineCurve3d::paintGL() {
	//genPoints("normal");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// 清除屏幕和深度缓存
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//平移

	glRotated(rotationX, 1.0, 0.0, 0.0);
	glRotated(rotationY, 0.0, 1.0, 0.0);
	glRotated(rotationZ, 0.0, 0.0, 1.0);//旋转
	// 绘制坐标轴
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0, 0.0, 1.0); // 蓝色表示x
	glVertex3d(0, 0, 0);
	glVertex3d(1000, 0, 0);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0, 1.0, 0.0);// 绿色  表示 y
	glVertex3d(0, 0, 0);
	glVertex3d(0, 1000, 0);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glColor3f(1.0, 0, 0.0);// 红色表示的是 Z
	glVertex3d(0, 0, 0);
	glVertex3d(0, 0, 1000);
	glEnd();
	// 画控制点
	glPointSize(8.0);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);
	for (int i = 0; i < bs3dConfig.ctrlPoints.size(); i++) {
		glVertex3d(bs3dConfig.ctrlPoints[i].x, bs3dConfig.ctrlPoints[i].y, bs3dConfig.ctrlPoints[i].z);
	}
	glEnd();

	// 画出控制多边形
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0, 1.0, 0.0);
	for (int i = 0; i < bs3dConfig.ctrlPoints.size(); i++) {
		glVertex3d(bs3dConfig.ctrlPoints[i].x, bs3dConfig.ctrlPoints[i].y, bs3dConfig.ctrlPoints[i].z);
	}
	glEnd();

	// 画出曲线
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0, 0.0, 1.0);
	for (int i = 0; i < bs3dConfig.curvaturePoints.size(); i++) {
		glVertex3f(bs3dConfig.curvaturePoints[i].x, bs3dConfig.curvaturePoints[i].y, bs3dConfig.curvaturePoints[i].z);
	}
	glEnd();
}



/**************************************************
@brief   : 按键按下事件
@author  : lee
@input   ：e  事件
@output  ：none
@time    : none
**************************************************/
void BsplineCurve3d::mousePressEvent(QMouseEvent *e) {
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
void BsplineCurve3d::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : none
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/
void BsplineCurve3d::mouseMoveEvent(QMouseEvent *e) {
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