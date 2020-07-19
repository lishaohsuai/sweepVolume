#include "nurbsCurve.h"
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
NurbsCurve::NurbsCurve(QWidget *parent) :
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
NurbsCurve::~NurbsCurve() {}

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
@brief   : ���������ϵĵ�
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
/* 
Ҳ�ǶԵķ������Ǻ����ϵ��㷨�е�����Ӧ���Ǵ���ʽ����Ч�ʲ��Ǻܸ߲���������ķ���
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
	// ���� ����� w
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
		w = w + base[i] * weight[span - p + i];// ���������˷�ĸ�ļ���
	}
	Point3d Cw;
	Cw.x = x; Cw.y = y; Cw.z = z; double Cww = w; // Cww �����ӵ�һ��ά��
	// ���� ����� w
	if (Cww == 0) {
		std::cout << "Cww== 0 " << Cww  << std::endl;
		exit(0);
	}
	C.x = Cw.x / Cww; C.y = Cw.y / Cww; C.z = Cw.z / Cww;
}
/**************************************************
@brief   : �򵥵Ľ����ɵĵ���뵽������
@author  : lee
@input   ��str  �ļ�·��
@output  ��none
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
		double u = i * 1.0 * (nurbsCvCfg.U_vector[nurbsCvCfg.U_vector.size() - 1] - nurbsCvCfg.U_vector[0]) / nurbsCvCfg.curvaturePointsNumber;//���ȷֳɶ��ٸ��������еĽڵ���
		Point3d C;
		curvePoint(nurbsCvCfg.ctrlPoints.size() - 1, nurbsCvCfg.X_Power, nurbsCvCfg.U_vector, nurbsCvCfg.ctrlPoints, u, C, nurbsCvCfg.weight);
		nurbsCvCfg.curvaturePoints.push_back(C);
	}
}



/**************************************************
@brief   : ���ļ��ж�ȡ��Ӧ�Ĳ���
@author  : lee
@input   ���ļ���(����·��)/��� �������
@output  ��none
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
@brief   : ��ʼ��opengl����
@author  : lee
@input   ��none
@output  ��none
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
@brief   : �ı䴰�ڵĴ�С
@author  : lee
@input   ��width ���
		   height �߶�
@output  ��none
@time    : none
**************************************************/
void NurbsCurve::resizeGL(int width, int height)
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
void NurbsCurve::paintGL() {
	//genPoints("normal");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// �����Ļ����Ȼ���
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//ƽ��

	glRotated(rotationX, 1.0, 0.0, 0.0);
	glRotated(rotationY, 0.0, 1.0, 0.0);
	glRotated(rotationZ, 0.0, 0.0, 1.0);//��ת

	// �����Ƶ�
	glPointSize(8.0);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);
	for (int i = 0; i < nurbsCvCfg.ctrlPoints.size(); i++) {
		glVertex3d(nurbsCvCfg.ctrlPoints[i].x, nurbsCvCfg.ctrlPoints[i].y, nurbsCvCfg.ctrlPoints[i].z);
	}
	glEnd();

	// �������ƶ����
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0, 1.0, 0.0);
	for (int i = 0; i < nurbsCvCfg.ctrlPoints.size(); i++) {
		glVertex3d(nurbsCvCfg.ctrlPoints[i].x, nurbsCvCfg.ctrlPoints[i].y, nurbsCvCfg.ctrlPoints[i].z);
	}
	glEnd();

	// ��������
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0, 0.0, 1.0);
	for (int i = 0; i < nurbsCvCfg.curvaturePoints.size(); i++) {
		glVertex3f(nurbsCvCfg.curvaturePoints[i].x, nurbsCvCfg.curvaturePoints[i].y, nurbsCvCfg.curvaturePoints[i].z);
	}
	glEnd();
}



/**************************************************
@brief   : ���������¼�
@author  : lee
@input   ��e  �¼�
@output  ��none
@time    : none
**************************************************/
void NurbsCurve::mousePressEvent(QMouseEvent *e) {
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
void NurbsCurve::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : none
@author  : none
@input   ��none
@output  ��none
@time    : none
**************************************************/
void NurbsCurve::mouseMoveEvent(QMouseEvent *e) {
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