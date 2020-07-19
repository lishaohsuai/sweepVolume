#include "bsplineSurface.h"
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
BsplineSurface3d::BsplineSurface3d(QWidget *parent) :
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
BsplineSurface3d::~BsplineSurface3d() {}

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
int BsplineSurface3d::findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U) {
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
void BsplineSurface3d::basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base) {
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
@brief   : ���������ϵĵ�  ��ʽ��
@author  : lee
@input   : n+1 ���Ƶ�ĸ���
		   p �״�
		   U �ڵ�����
			m+1 �ڵ�ĸ���
		   m+1 ��һ�����Ƶ�ĸ���
		   q ��һ���״�
		   V ��һ���ڵ�����
		   P  ���Ƶ����� ��ά����
		   u��v ��С���󣬴Ӷ����Ƴ���������
		   S ���ɵĵ������
@output  ��none
@time    : none
**************************************************/
void BsplineSurface3d::surfacePoint(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V, 
		std::vector<std::vector<Point3d> >&P, double u, double v, Point3d &S) {
	int uspan = findSpan(n, p, u, U);
	std::vector<double> baseu;
	basisFuns(uspan, u, p, U, baseu);
	std::vector<double> basev;
	int vspan = findSpan(m, q, v, V);
	basisFuns(vspan, v, q, V, basev);
	int uind = uspan - p;
	S.x = S.y = S.z = 0;
	for (int l = 0; l <= q; l++) {
		Point3d temp(0, 0, 0);
		int vind = vspan - q + l;
		for (int k = 0; k <= p; k++) {
			temp.x = temp.x + baseu[k] * P[uind + k][vind].x;
			temp.y = temp.y + baseu[k] * P[uind + k][vind].y;
			temp.z = temp.z + baseu[k] * P[uind + k][vind].z;
		}
		S.x = S.x + basev[l] * temp.x;
		S.y = S.y + basev[l] * temp.y;
		S.z = S.z + basev[l] * temp.z;
	}
}

/**************************************************
@brief   : �򵥵Ľ����ɵĵ���뵽������
@author  : lee
@input   ��str  �ļ�·��
@output  ��none
@time    : none
**************************************************/
void BsplineSurface3d::genPoints(std::string str) {
	jsonReader(str);
	std::cout << "[DEBUG]genPoint" << std::endl;
	for (int i = 0; i < bsSf3dCfg.surfacePoints.size(); i++) {
		bsSf3dCfg.surfacePoints[i].clear();
	}
	bsSf3dCfg.surfacePoints.clear();
	
	for (int i = 0; i < bsSf3dCfg.surfacePointsNumberRow; i++) {
		double u = i * 1.0 * (bsSf3dCfg.U_vector[bsSf3dCfg.U_vector.size() - 1] - bsSf3dCfg.U_vector[0]) / bsSf3dCfg.surfacePointsNumberRow;//���ȷֳɶ��ٸ��������еĽڵ���
		std::vector<Point3d> temp;
		for (int j = 0; j < bsSf3dCfg.surfacePointsNumberCol; j++) {
			double v = j * 1.0 * (bsSf3dCfg.V_vector[bsSf3dCfg.V_vector.size() - 1] - bsSf3dCfg.V_vector[0]) / bsSf3dCfg.surfacePointsNumberCol;//���ȷֳɶ��ٸ��������еĽڵ���
			Point3d S;
			surfacePoint(bsSf3dCfg.ctrlPoints.size() - 1, bsSf3dCfg.P_Power, bsSf3dCfg.U_vector, bsSf3dCfg.ctrlPoints[0].size() - 1, bsSf3dCfg.Q_Power
				, bsSf3dCfg.V_vector, bsSf3dCfg.ctrlPoints, u, v, S);
			temp.push_back(S);
		}
		bsSf3dCfg.surfacePoints.push_back(temp);
	}
}



/**************************************************
@brief   : ���ļ��ж�ȡ��Ӧ�Ĳ���
@author  : lee
@input   ���ļ���(����·��)/��� �������
@output  ��none
@time    : none
**************************************************/
void BsplineSurface3d::jsonReader(std::string fileName) {
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
		bsSf3dCfg.name = root["name"].asString();
		bsSf3dCfg.P_Power = root["P_Power"].asInt();
		bsSf3dCfg.Q_Power = root["Q_Power"].asInt();
		bsSf3dCfg.surfacePointsNumberCol = root["surfacePointsNumberCol"].asInt();
		bsSf3dCfg.surfacePointsNumberRow = root["surfacePointsNumberRow"].asInt();
		Json::Value points = root["ctrlPoint"];
		for (int i = 0; i < points.size(); i++) {
			std::vector<Point3d> temp;
			for (int j = 0; j < points[0].size(); j++) {
				Point3d p(0, 0, 0);
				p.x = (points[i][j][0].asDouble());
				p.y = (points[i][j][1].asDouble());
				p.z = (points[i][j][2].asDouble());
				temp.push_back(p);
			}
			bsSf3dCfg.ctrlPoints.push_back(temp);
		}
		Json::Value vectorsU = root["U_vector"];
		for (int i = 0; i < vectorsU.size(); i++) {
			bsSf3dCfg.U_vector.push_back(vectorsU[i].asDouble());
		}
		Json::Value vectorsV = root["V_vector"];
		for (int i = 0; i < vectorsV.size(); i++) {
			bsSf3dCfg.V_vector.push_back(vectorsV[i].asDouble());
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
void BsplineSurface3d::initializeGL() {
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
void BsplineSurface3d::resizeGL(int width, int height)
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
void BsplineSurface3d::paintGL() {
	//genPoints("normal");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// �����Ļ����Ȼ���
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//ƽ��

	glRotated(rotationX, 1.0, 0.0, 0.0);
	glRotated(rotationY, 0.0, 1.0, 0.0);
	glRotated(rotationZ, 0.0, 0.0, 1.0);//��ת
	// ����������
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0, 0.0, 1.0); // ��ɫ��ʾx
	glVertex3d(0, 0, 0);
	glVertex3d(1000, 0, 0);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0, 1.0, 0.0);// ��ɫ  ��ʾ y
	glVertex3d(0, 0, 0);
	glVertex3d(0, 1000, 0);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glColor3f(1.0, 0, 0.0);// ��ɫ��ʾ���� Z
	glVertex3d(0, 0, 0);
	glVertex3d(0, 0, 1000);
	glEnd();
	// �����Ƶ�
	glPointSize(8.0);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);
	for (int i = 0; i < bsSf3dCfg.ctrlPoints.size(); i++) {
		for (int j = 0; j < bsSf3dCfg.ctrlPoints[0].size(); j++) {
			glVertex3d(bsSf3dCfg.ctrlPoints[i][j].x, bsSf3dCfg.ctrlPoints[i][j].y, bsSf3dCfg.ctrlPoints[i][j].z);
		}
	}
	glEnd();


	// ������������
	for (int i = 0; i < bsSf3dCfg.surfacePoints.size(); i++) {
		glBegin(GL_LINE_STRIP);
		glColor3f(1.0, 1.0, 1.0);
		for (int j = 0; j < bsSf3dCfg.surfacePoints[0].size(); j++) {
			glVertex3d(bsSf3dCfg.surfacePoints[i][j].x, bsSf3dCfg.surfacePoints[i][j].y, bsSf3dCfg.surfacePoints[i][j].z);
		}
		glEnd();
	}
	for (int i = 0; i < bsSf3dCfg.surfacePoints[0].size(); i++) {
		glBegin(GL_LINE_STRIP);
		glColor3f(1.0, 1.0, 1.0);
		for (int j = 0; j < bsSf3dCfg.surfacePoints.size(); j++) {
			glVertex3d(bsSf3dCfg.surfacePoints[j][i].x, bsSf3dCfg.surfacePoints[j][i].y, bsSf3dCfg.surfacePoints[j][i].z);
		}
		glEnd();
	}
}



/**************************************************
@brief   : ���������¼�
@author  : lee
@input   ��e  �¼�
@output  ��none
@time    : none
**************************************************/
void BsplineSurface3d::mousePressEvent(QMouseEvent *e) {
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
void BsplineSurface3d::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : none
@author  : none
@input   ��none
@output  ��none
@time    : none
**************************************************/
void BsplineSurface3d::mouseMoveEvent(QMouseEvent *e) {
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