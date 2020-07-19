#include "SweptSurfaceSilt.h"
#include <QFileDialog>
#include <fstream>
#include <QGLWidget>
#include <gl/glu.h>

/**************************************************
@brief   : ɨ�����ʵ�ֲ���
			 ����������Nurbs����
			 ����v��������u�˶�
@author  : none
@input   ��none
@output  ��none
@time    : none
**************************************************/

/**************************************************
@brief   : ���캯�� �򵥵ĳ�ʼ��������ֵ
@author  : lee
@input   ��parent һ��ΪNULL
@output  ��none
@time    : none
**************************************************/
SweptSurfaceTransformSilt::SweptSurfaceTransformSilt(QWidget *parent) :
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
SweptSurfaceTransformSilt::~SweptSurfaceTransformSilt() {}

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
int SweptSurfaceTransformSilt::findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U) {
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
void SweptSurfaceTransformSilt::basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base) {
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
void SweptSurfaceTransformSilt::curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u,
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
		w = w + base[i] * weight[span - p + i];
	}
	Point3d Cw;
	Cw.x = x; Cw.y = y; Cw.z = z; double Cww = w; // Cww �����ӵ�һ��ά��
	// ���� ����� w
	if (Cww == 0) {
		std::cout << "Cww== 0 " << Cww << std::endl;
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
void SweptSurfaceTransformSilt::genPoints(std::string str) {
	jsonReader(str);
	std::cout << "[DEBUG]genPoint" << std::endl;
	vector3d Bv(1, 0, 0);
	Matrix33 identity = Matrix33::getIdentityMatrix();
	sweptEntity.getSweepSurfaceSilt(T, C, S, 1000);// �õ������������ɵ�һ������Ľṹ��
	sweptEntity.outputSweptSur(cfg.P_Power, cfg.Q_Power, cfg.U_vector, cfg.V_vector, cfg.ctrlPoints);//�������ṹ�壬Ȼ�����ƽ��
	cfg.surfacePointsNumberRow = 50;
	cfg.surfacePointsNumberCol = 50;
	for (int i = 0; i < cfg.surfacePointsNumberRow; i++) {
		double u = i * 1.0 * (cfg.U_vector[cfg.U_vector.size() - 1] - cfg.U_vector[0]) / cfg.surfacePointsNumberRow;//���ȷֳɶ��ٸ��������еĽڵ���
		std::vector<Point3d> temp;
		for (int j = 0; j < cfg.surfacePointsNumberCol; j++) {
			double v = j * 1.0 * (cfg.V_vector[cfg.V_vector.size() - 1] - cfg.V_vector[0]) / cfg.surfacePointsNumberCol;//���ȷֳɶ��ٸ��������еĽڵ���
			Point3d S;
			surfacePoint(cfg.ctrlPoints.size() - 1, cfg.P_Power, cfg.U_vector, cfg.ctrlPoints[0].size() - 1, cfg.Q_Power
				, cfg.V_vector, cfg.ctrlPoints, u, v, S);
			temp.push_back(S);
		}
		cfg.surfacePoints.push_back(temp);
	}
	
	for (int i = 0; i < 1000; i++) {
		double u = i * 1.0 * (cfg.U_vector[cfg.U_vector.size() - 1] - cfg.U_vector[0]) / 1000;//
		curvePoints.push_back(T.getPointAt(u));
	}
}

/**************************************************
@brief   : ���������ϵĵ�
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
void SweptSurfaceTransformSilt::surfacePoint(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V,
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
@brief   : ���ļ��ж�ȡ��Ӧ�Ĳ���
@author  : lee
@input   ���ļ���(����·��)/��� �������
@output  ��none
@time    : none
**************************************************/
void SweptSurfaceTransformSilt::jsonReader(std::string fileName) {
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
		nurbsDoCvCfg.name = root["name"].asString();
		nurbsDoCvCfg.P_Power = root["P_Power"].asInt();
		nurbsDoCvCfg.Q_Power = root["Q_Power"].asInt();
		nurbsDoCvCfg.curvaturePointsNumber1 = root["trajectoryPointNum"].asInt();
		nurbsDoCvCfg.curvaturePointsNumber2 = root["sectionPointNum"].asInt();
		nurbsDoCvCfg.surfacePointsNum = root["surfacePointsNum"].asInt();
		Json::Value points1 = root["trajectoryCtrlPoint"];
		for (int i = 0; i < points1.size(); i++) {
			Point3d p(0, 0, 0);
			p.x = (points1[i][0].asDouble());
			p.y = (points1[i][1].asDouble());
			p.z = (points1[i][2].asDouble());
			nurbsDoCvCfg.ctrlPoints1.push_back(p);
		}
		Json::Value points2 = root["sectionCtrlPoint"];
		for (int i = 0; i < points2.size(); i++) {
			Point3d p(0, 0, 0);
			p.x = (points2[i][0].asDouble());
			p.y = (points2[i][1].asDouble());
			p.z = (points2[i][2].asDouble());
			nurbsDoCvCfg.ctrlPoints2.push_back(p);
		}
		Json::Value vectors = root["U_vector"];
		for (int i = 0; i < vectors.size(); i++) {
			nurbsDoCvCfg.U_vector.push_back(vectors[i].asDouble());
		}
		Json::Value vectors1 = root["V_vector"];
		for (int i = 0; i < vectors1.size(); i++) {
			nurbsDoCvCfg.V_vector.push_back(vectors1[i].asDouble());
		}
		Json::Value weight1 = root["trajectoryWeight"];
		for (int i = 0; i < weight1.size(); i++) {
			nurbsDoCvCfg.weight1.push_back(weight1[i].asDouble());
		}
		Json::Value weight2 = root["sectionWeight"];
		for (int i = 0; i < weight2.size(); i++) {
			nurbsDoCvCfg.weight2.push_back(weight2[i].asDouble());
		}
		// ʹ�� �ṹ���ʼ���������ߵĽṹ�� (int p_, int n_, std::vector<double> U_, std::vector<Point3d> controlPts_) :
		BSplineCurveAbstract tmp(nurbsDoCvCfg.P_Power, nurbsDoCvCfg.ctrlPoints1.size() - 1, nurbsDoCvCfg.U_vector, nurbsDoCvCfg.ctrlPoints1);
		T.set(tmp);
	
		// move the section curve to origin // QU:ΪʲôҪ��section Curve �ƶ���ԭ��
		Point3d translation(nurbsDoCvCfg.ctrlPoints2[0]);
		for (int i = 0; i < nurbsDoCvCfg.ctrlPoints2.size(); i++) {
			nurbsDoCvCfg.ctrlPoints2[i] = nurbsDoCvCfg.ctrlPoints2[i] - translation;
		}
		BSplineCurveAbstract temp(nurbsDoCvCfg.Q_Power, nurbsDoCvCfg.ctrlPoints2.size() - 1, nurbsDoCvCfg.V_vector, nurbsDoCvCfg.ctrlPoints2);
		C.set(temp);
		// scale 
		Json::Value vectorsScale = root["scale_vector"];
		for (int i = 0; i < vectorsScale.size(); i++) {
			scaleCurve.U_vector.push_back(vectorsScale[i].asDouble());
		}
		Json::Value points3 = root["scaleCtrlPoint"];
		for (int i = 0; i < points3.size(); i++) {
			Point3d p(0, 0, 0);
			p.x = (points3[i][0].asDouble());
			p.y = (points3[i][1].asDouble());
			p.z = (points3[i][2].asDouble());
			scaleCurve.ctrlPoints.push_back(p);
		}
		scaleCurve.X_Power = root["scale_Power"].asInt();
		BSplineCurveAbstract temp1(scaleCurve.X_Power, scaleCurve.ctrlPoints.size() - 1, scaleCurve.U_vector, scaleCurve.ctrlPoints);
		S.set(temp1);
	}

}


/**************************************************
@brief   : ��ʼ��opengl����
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/
void SweptSurfaceTransformSilt::initializeGL() {
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
void SweptSurfaceTransformSilt::resizeGL(int width, int height)
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
@input   : none
@output  : none
@time    : none
**************************************************/
void SweptSurfaceTransformSilt::paintGL() {
	//genPoints("normal");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// �����Ļ����Ȼ���
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//ƽ��

	glRotated(rotationX, 1.0, 0.0, 0.0);
	glRotated(rotationY, 0.0, 1.0, 0.0);
	glRotated(rotationZ, 0.0, 0.0, 1.0);//��ת

	// �����Ƶ�
	//glPointSize(8.0);
	//glBegin(GL_POINTS);
	//glColor3f(1.0, 0.0, 0.0);
	//for (int i = 0; i < cfg.ctrlPoints.size(); i++) {
	//	for (int j = 0; j < cfg.ctrlPoints[0].size(); j++) {
	//		glVertex3d(cfg.ctrlPoints[i][j].x, cfg.ctrlPoints[i][j].y, cfg.ctrlPoints[i][j].z);
	//	}
	//}
	//glEnd();
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
	// ����ɨ����
	glBegin(GL_LINE_STRIP);
	glColor3f(1.0, 1.0, 0.0);
	for (int i = 0; i < curvePoints.size(); i++) {
		glVertex3d(curvePoints[i].x, curvePoints[i].y, curvePoints[i].z);

	}
	glEnd();

	// ������������
	for (int i = 0; i < cfg.surfacePoints.size(); i++) {
		glBegin(GL_LINE_STRIP);
		glColor3f(1.0, 1.0, 1.0);
		for (int j = 0; j < cfg.surfacePoints[0].size(); j++) {
			glVertex3d(cfg.surfacePoints[i][j].x, cfg.surfacePoints[i][j].y, cfg.surfacePoints[i][j].z);
		}
		glEnd();
	}
	for (int i = 0; i < cfg.surfacePoints[0].size(); i++) {
		glBegin(GL_LINE_STRIP);
		glColor3f(1.0, 1.0, 1.0);
		for (int j = 0; j < cfg.surfacePoints.size(); j++) {
			glVertex3d(cfg.surfacePoints[j][i].x, cfg.surfacePoints[j][i].y, cfg.surfacePoints[j][i].z);
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
void SweptSurfaceTransformSilt::mousePressEvent(QMouseEvent *e) {
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
void SweptSurfaceTransformSilt::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : none
@author  : none
@input   ��none
@output  ��none
@time    : none
**************************************************/
void SweptSurfaceTransformSilt::mouseMoveEvent(QMouseEvent *e) {
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


