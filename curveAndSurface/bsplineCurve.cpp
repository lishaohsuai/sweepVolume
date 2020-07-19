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
	precision = 5;//���ѡ��ľ���Ϊ5
};

BsplineCurve::~BsplineCurve() {}




/**************************************************
@brief   : ���ļ��ж�ȡ��Ӧ�Ĳ���
@author  : lee
@input   ���ļ���(����·��)/��� �������
@output  ��none
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
@brief   : �򵥵Ľ����ɵĵ���뵽������
@author  : lee
@input   ��str  �ļ�·��
@output  ��none
@time    : none 
**************************************************/
void BsplineCurve::genPoints(std::string str) {
	if(str != "")
		jsonReader(str);
	std::cout << "[DEBUG]genPoint" << std::endl;
	bsConfig.curvaturePoints.clear();
	for (int i = 0; i <= bsConfig.curvaturePointsNumber; i++) {
		double u = i * 1.0 * (bsConfig.U_vector[bsConfig.U_vector.size() - 1] - bsConfig.U_vector[0]) / bsConfig.curvaturePointsNumber;//���ȷֳɶ��ٸ��������еĽڵ���
		QPointF C;
		curvePoint(bsConfig.ctrlPoints.size() - 1, bsConfig.X_Power, bsConfig.U_vector, bsConfig.ctrlPoints, u, C);
		bsConfig.curvaturePoints.push_back(C);
	}
}

/**************************************************
@brief   : �ҵ� u �����ڵ���������
@author  : lee
@input   : numOfCtlPoint ���Ƶ�ĸ���
		   order �������Ľ�
		   u ������һ���0��1�ϵĵ�
		   U �ڵ�����
@output  : �ڵ���������
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
@brief   : ����ǿյĻ����� 
		   ��һ�Σ�p=0ʱ��ֻ��N[0]=1.0,��N0,0=1.0;p=1ʱ����N[0],N[1],��N0,1��N1,1;p=2ʱ����N[0],N[1],N[2],��N0,2  N1,2��N2,2
@author  : lee
@input   ��i �ڵ������ĵ�i������
		   u ���������ϵĵ�
		   order �״�
		   U �ڵ�����
@output  ��base �ӵ�0�׵�����Ľ׷��������������Ӧ�Ľ׺�����ֵ��һ����
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
@brief   : ���������ϵĵ� ��ʽ��
		   �ؼ���ʽ  m = n + p + 1
@author  : lee
@input   : n+1 ���Ƶ�ĸ���
		   p �״�
		   m+1 �ڵ�ĸ���
		   P  ���Ƶ�����
		   u ��С���󣬴Ӷ����Ƴ���������
		   C ���ɵĵ������
@output  ��none
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
@input   ��none
@output  ��none
@time    : none
**************************************************/
void BsplineCurve::paintEvent(QPaintEvent *) {
	//genPoints("normal");

	QPainter painter(this);
	QPen pen;

	// �����Ƶ�
	pen.setWidth(8);
	pen.setColor(Qt::blue);
	painter.setPen(pen);
	for (int i = 0; i < bsConfig.ctrlPoints.size(); i++) {
		painter.drawPoint(bsConfig.ctrlPoints[i]);
	}

	// �������ƶ����
	pen.setWidth(3);
	pen.setColor(Qt::black);
	painter.setPen(pen);
	for (int i = 0; i < bsConfig.ctrlPoints.size() - 1; i++) {
		painter.drawLine(bsConfig.ctrlPoints[i], bsConfig.ctrlPoints[i + 1]);
	}

	// ��������
	pen.setWidth(2);
	pen.setColor(Qt::green);
	painter.setPen(pen);
	for (int i = 0; i < bsConfig.curvaturePointsNumber; i++) {
		painter.drawLine(bsConfig.curvaturePoints[i], bsConfig.curvaturePoints[i + 1]);
	}
}


/**************************************************
@brief   : �ı䴰�ڵĴ�С
@author  : lee
@input   ��width ���
		   height �߶�
@output  ��none
@time    : none
**************************************************/
void BsplineCurve::resizeGL(int width, int height)
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
@brief   : ʵ�ְ������ͷ�
@author  : lee
@input   ��event
@output  ��none
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
@brief   : ʵ�ְ����İ�ѹ ͨ���жϰ��µİ���������һ�����Ƶ�ȽϽ�Ȼ��ѡ��������Ƶ㣬
@author  : lee
@input   ��event
@output  ��none
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
@brief   : ʵ�ְ������ƶ�
@author  : lee
@input   ��event
@output  ��none
@time    : none
**************************************************/
void BsplineCurve::mouseMoveEvent(QMouseEvent *event) {
	if (flag != -1) {
		bsConfig.ctrlPoints[flag] = event->pos();
		//this->update();
	}
}