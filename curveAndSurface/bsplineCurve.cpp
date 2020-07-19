#include "bsplineCurve.h"
#include <QFileDialog>
#include <fstream>
#include <QPen>
#include <QPainter>
#include <QWidget.h>
#include <QGLWidget>
#include <gl/glu.h>


BsplineCurve::BsplineCurve(QWidget *parent) :
	Strategy(), QWidget(parent) {
	flag = -1;
	precision = 5;//鼠标选点的精度为5
};

BsplineCurve::~BsplineCurve() {}




/**************************************************
@brief   : 从文件中读取相应的参数
@author  : lee
@input   ：文件名(完整路径)/或空 方便调试
@output  ：none
@time    : none
**************************************************/
void BsplineCurve::jsonReader(std::string fileName) {
	if (fileName == "") {
		QString QfileName = QFileDialog::getOpenFileName(NULL,
			QObject::tr("Open json file"),
			QObject::tr("./config"),
			QObject::tr("mesh (*.json);;"
				"All Files (*)"));
		std::string fileName = QfileName.toStdString();
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
		bsConfig.name = root["name"].asString();
		bsConfig.X_Power = root["X_Power"].asInt();
		bsConfig.curvaturePointsNumber = root["curvePointNum"].asInt();
		Json::Value points = root["ctrlPoint"];
		for (int i = 0; i < points.size(); i++) {
			QPointF p(0, 0);
			p.setX(points[i][0].asDouble());
			p.setY(points[i][1].asDouble());
			bsConfig.ctrlPoints.push_back(p);
		}
		Json::Value vectors = root["U_vector"];
		for (int i = 0; i < vectors.size(); i++) {
			bsConfig.U_vector.push_back(vectors[i].asDouble());
		}
	}
}



/**************************************************
@brief   : 简单的讲生成的点加入到曲线上
@author  : lee
@input   ：str  文件路径
@output  ：none
@time    : none 
**************************************************/
void BsplineCurve::genPoints(std::string str) {
	if(str != "")
		jsonReader(str);
	std::cout << "[DEBUG]genPoint" << std::endl;
	bsConfig.curvaturePoints.clear();
	for (int i = 0; i <= bsConfig.curvaturePointsNumber; i++) {
		double u = i * 1.0 * (bsConfig.U_vector[bsConfig.U_vector.size() - 1] - bsConfig.U_vector[0]) / bsConfig.curvaturePointsNumber;//均匀分成多少个点再所有的节点上
		QPointF C;
		curvePoint(bsConfig.ctrlPoints.size() - 1, bsConfig.X_Power, bsConfig.U_vector, bsConfig.ctrlPoints, u, C);
		bsConfig.curvaturePoints.push_back(C);
	}
}

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
int BsplineCurve::findSpan(int numOfCtlPoint, int order, double u,const std::vector<double> &U) {
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
void BsplineCurve::basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base) {
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
@brief   : 生成曲线上的点 公式法
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
void BsplineCurve::curvePoint(int n, int p, const std::vector<double>&U, std::vector<QPointF>&P, double u, QPointF &C) {
	int span = findSpan(n, p, u, U);
	std::vector<double> base;
	basisFuns(span, u, p, U, base);
	double x, y;
	x = y = 0;
	for (int i = 0; i <= p; i++) {
		x = x + base[i] * P[span - p + i].x();
		y = y + base[i] * P[span - p + i].y();
	}
	//std::cout << "u x y-" << u << " " << x << " " << y << std::endl;
	QPointF temp;
	temp.setX(x);
	temp.setY(y);
	C = temp;
}


/**************************************************
@brief   : none
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/
void BsplineCurve::paintEvent(QPaintEvent *) {
	//genPoints("normal");

	QPainter painter(this);
	QPen pen;

	// 画控制点
	pen.setWidth(8);
	pen.setColor(Qt::blue);
	painter.setPen(pen);
	for (int i = 0; i < bsConfig.ctrlPoints.size(); i++) {
		painter.drawPoint(bsConfig.ctrlPoints[i]);
	}

	// 画出控制多边形
	pen.setWidth(3);
	pen.setColor(Qt::black);
	painter.setPen(pen);
	for (int i = 0; i < bsConfig.ctrlPoints.size() - 1; i++) {
		painter.drawLine(bsConfig.ctrlPoints[i], bsConfig.ctrlPoints[i + 1]);
	}

	// 画出曲线
	pen.setWidth(2);
	pen.setColor(Qt::green);
	painter.setPen(pen);
	for (int i = 0; i < bsConfig.curvaturePointsNumber; i++) {
		painter.drawLine(bsConfig.curvaturePoints[i], bsConfig.curvaturePoints[i + 1]);
	}
}


/**************************************************
@brief   : 改变窗口的大小
@author  : lee
@input   ：width 宽度
		   height 高度
@output  ：none
@time    : none
**************************************************/
void BsplineCurve::resizeGL(int width, int height)
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
@brief   : 实现按键的释放
@author  : lee
@input   ：event
@output  ：none
@time    : none
**************************************************/
void BsplineCurve::mouseReleaseEvent(QMouseEvent *event) {
	if (flag != -1) {
		genPoints("");
		update();
	}
	flag = -1;
}


/**************************************************
@brief   : 实现按键的按压 通过判断按下的按键和离哪一个控制点比较近然后选中这个控制点，
@author  : lee
@input   ：event
@output  ：none
@time    : none
**************************************************/
void BsplineCurve::mousePressEvent(QMouseEvent *event) {
	flag = -1;
	QPointF tempPoint = event->pos();
	for (int i = 0; i < bsConfig.ctrlPoints.size(); i++) {
		double x = bsConfig.ctrlPoints[i].x();
		double y = bsConfig.ctrlPoints[i].y();
		if ((tempPoint.x() >= x - precision && tempPoint.x() <= x + precision) && (tempPoint.y() >= y - precision && tempPoint.y() <= y + precision)) {
			flag = i;
			break;
		}
	}
}


/**************************************************
@brief   : 实现按键的移动
@author  : lee
@input   ：event
@output  ：none
@time    : none
**************************************************/
void BsplineCurve::mouseMoveEvent(QMouseEvent *event) {
	if (flag != -1) {
		bsConfig.ctrlPoints[flag] = event->pos();
		//this->update();
	}
}