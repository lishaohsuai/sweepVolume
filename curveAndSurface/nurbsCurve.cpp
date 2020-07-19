#include "nurbsCurve.h"
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
NurbsCurve::NurbsCurve(QWidget *parent) :
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
NurbsCurve::~NurbsCurve() {}

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
int NurbsCurve::findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U) {
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
void NurbsCurve::basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base) {
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
/* 
也是对的方法但是和书上的算法有点区别，应该是纯公式法。效率不是很高采用了下面的方法
void NurbsCurve::curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u, 
		Point3d &C, const std::vector<double> &weight) {
	int span = findSpan(n, p, u, U);
	std::vector<double> base;
	basisFuns(span, u, p, U, base);
	double x, y, z;
	x = y = z = 0;
	for (int i = 0; i <= p; i++) {
		x = x + base[i] * P[span - p + i].x * weight[span - p + i];
		y = y + base[i] * P[span - p + i].y * weight[span - p + i];
		z = z + base[i] * P[span - p + i].z * weight[span - p + i];
	}
	Point3d Cw;
	Cw.x = x; Cw.y = y; Cw.z = z;
	// 计算 下面的 w
	C.x = 0; C.y = 0; C.z = 0;
	for (int i = 0; i <= p; i++) {
		C.x = C.x + base[i] * weight[span - p + i];
		C.y = C.y + base[i] * weight[span - p + i];
		C.z = C.z + base[i] * weight[span - p + i];
	}
	if (C.x == 0 || C.y == 0 || C.z == 0) {
		std::cout << "C.xyz == 0 " << C.x << " " << C.y << " " << C.z << std::endl;
		exit(0);
	}
	C.x = Cw.x / C.x; C.y = Cw.y / C.y; C.z = Cw.z / C.z;
}*/
void NurbsCurve::curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u,
	Point3d &C, const std::vector<double> &weight) {
	int span = findSpan(n, p, u, U);
	std::vector<double> base;
	basisFuns(span, u, p, U, base);
	double x, y, z, w;
	x = y = z = w = 0;
	for (int i = 0; i <= p; i++) {
		x = x + base[i] * P[span - p + i].x * weight[span - p + i];
		y = y + base[i] * P[span - p + i].y * weight[span - p + i];
		z = z + base[i] * P[span - p + i].z * weight[span - p + i];
		w = w + base[i] * weight[span - p + i];// 这里体现了分母的计算
	}
	Point3d Cw;
	Cw.x = x; Cw.y = y; Cw.z = z; double Cww = w; // Cww 新增加的一个维度
	// 计算 下面的 w
	if (Cww == 0) {
		std::cout << "Cww== 0 " << Cww  << std::endl;
		exit(0);
	}
	C.x = Cw.x / Cww; C.y = Cw.y / Cww; C.z = Cw.z / Cww;
}
/**************************************************
@brief   : 简单的讲生成的点加入到曲线上
@author  : lee
@input   ：str  文件路径
@output  ：none
@time    : none
**************************************************/
void NurbsCurve::genPoints(std::string str) {
	if (str != "")
		jsonReader(str);
	else
		jsonReader("");
	std::cout << "[DEBUG]genPoint" << std::endl;
	nurbsCvCfg.curvaturePoints.clear();
	for (int i = 0; i <= nurbsCvCfg.curvaturePointsNumber; i++) {
		double u = i * 1.0 * (nurbsCvCfg.U_vector[nurbsCvCfg.U_vector.size() - 1] - nurbsCvCfg.U_vector[0]) / nurbsCvCfg.curvaturePointsNumber;//均匀分成多少个点再所有的节点上
		Point3d C;
		curvePoint(nurbsCvCfg.ctrlPoints.size() - 1, nurbsCvCfg.X_Power, nurbsCvCfg.U_vector, nurbsCvCfg.ctrlPoints, u, C, nurbsCvCfg.weight);
		nurbsCvCfg.curvaturePoints.push_back(C);
	}
}



/**************************************************
@brief   : 从文件中读取相应的参数
@author  : lee
@input   ：文件名(完整路径)/或空 方便调试
@output  ：none
@time    : none
**************************************************/
void NurbsCurve::jsonReader(std::string fileName) {
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
		nurbsCvCfg.name = root["name"].asString();
		nurbsCvCfg.X_Power = root["X_Power"].asInt();
		nurbsCvCfg.curvaturePointsNumber = root["curvePointNum"].asInt();
		Json::Value points = root["ctrlPoint"];
		for (int i = 0; i < points.size(); i++) {
			Point3d p(0, 0, 0);
			p.x = (points[i][0].asDouble());
			p.y = (points[i][1].asDouble());
			p.z = (points[i][2].asDouble());
			nurbsCvCfg.ctrlPoints.push_back(p);
		}
		Json::Value vectors = root["U_vector"];
		for (int i = 0; i < vectors.size(); i++) {
			nurbsCvCfg.U_vector.push_back(vectors[i].asDouble());
		}
		Json::Value weight = root["weight"];
		for (int i = 0; i < weight.size(); i++) {
			nurbsCvCfg.weight.push_back(weight[i].asDouble());
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
void NurbsCurve::initializeGL() {
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
void NurbsCurve::resizeGL(int width, int height)
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
void NurbsCurve::paintGL() {
	//genPoints("normal");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// 清除屏幕和深度缓存
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//平移

	glRotated(rotationX, 1.0, 0.0, 0.0);
	glRotated(rotationY, 0.0, 1.0, 0.0);
	glRotated(rotationZ, 0.0, 0.0, 1.0);//旋转

	// 画控制点
	glPointSize(8.0);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);
	for (int i = 0; i < nurbsCvCfg.ctrlPoints.size(); i++) {
		glVertex3d(nurbsCvCfg.ctrlPoints[i].x, nurbsCvCfg.ctrlPoints[i].y, nurbsCvCfg.ctrlPoints[i].z);
	}
	glEnd();

	// 画出控制多边形
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0, 1.0, 0.0);
	for (int i = 0; i < nurbsCvCfg.ctrlPoints.size(); i++) {
		glVertex3d(nurbsCvCfg.ctrlPoints[i].x, nurbsCvCfg.ctrlPoints[i].y, nurbsCvCfg.ctrlPoints[i].z);
	}
	glEnd();

	// 画出曲线
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0, 0.0, 1.0);
	for (int i = 0; i < nurbsCvCfg.curvaturePoints.size(); i++) {
		glVertex3f(nurbsCvCfg.curvaturePoints[i].x, nurbsCvCfg.curvaturePoints[i].y, nurbsCvCfg.curvaturePoints[i].z);
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
void NurbsCurve::mousePressEvent(QMouseEvent *e) {
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
void NurbsCurve::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : none
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/
void NurbsCurve::mouseMoveEvent(QMouseEvent *e) {
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