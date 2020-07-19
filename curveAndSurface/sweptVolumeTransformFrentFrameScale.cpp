#include "SweptVolumeTransformFrentFrameScale.h"
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
SweptVolumeTransformFrentFrameScale::SweptVolumeTransformFrentFrameScale(QWidget *parent) :
	Strategy(), QGLWidget(parent) {
	flag = -1;// û���κε�����¼�
	rotationX = 0;
	rotationY = 0;
	rotationZ = 0;
	times = 1.0;
	Planar = false;
};



/**************************************************
@brief   : ���������ͷ������Դ
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/
SweptVolumeTransformFrentFrameScale::~SweptVolumeTransformFrentFrameScale() {}

/**************************************************
@brief   : �ҵ� u �����ڵ���������
@author  : lee
@input   : n n = m - p -1
		   p �������Ľ�
		   u ������һ���0��1�ϵĵ�
		   U �ڵ����� U_0 - U_m
@output  : �ڵ���������
@time    : none
**************************************************/
int SweptVolumeTransformFrentFrameScale::findSpan(int n, int p, double u, const std::vector<double> &U) {
	if (u >= U[n + 1]) return n;
	for (int i = p; i < n + 1; i++) {
		if (U[i] <= u && u < U[i + 1]) {
			return i;
		}
	}
}

/**************************************************
@brief   : ����ǿյĻ�����
		   ��һ�Σ�p=0ʱ��ֻ��N[0]=1.0,��N0,0=1.0;p=1ʱ����N[0],N[1],��N0,1��N1,1;p=2ʱ����N[0],N[1],N[2],��N0,2  N1,2��N2,2
@author  : lee
@input   ��i �ڵ������ĵ�i������
		   u ���������ϵĵ�
		   p �״�
		   U �ڵ�����
@output  ��N �ӵ�0�׵�����Ľ׷��������������Ӧ�Ľ׺�����ֵ��һ����
@time    : none
**************************************************/
void SweptVolumeTransformFrentFrameScale::basisFuns(int i, double u, int p, const std::vector<double> &U, std::vector<double> &N) {
	N.resize(p + 1);
	N[0] = 1.0;
	std::vector<double> left, right;
	left.resize(p + 1);
	right.resize(p + 1);
	for (int j = 1; j <= p; j++) {
		left[j] = u - U[i + 1 - j];
		right[j] = U[i + j] - u;
		double saved = 0.0;
		for (int r = 0; r < j; r++) {
			double temp = N[r] / (right[r + 1] + left[j - r]);
			N[r] = saved + right[r + 1] * temp;
			saved = left[j - r] * temp;
		}
		N[j] = saved;
	}
}


/**************************************************
@brief   : �򵥵Ľ����ɵĵ���뵽������
@author  : lee
@input   ��str  �ļ�·��
@output  ��none
@time    : none
**************************************************/
void SweptVolumeTransformFrentFrameScale::genPoints(std::string str) {
	jsonReader(str);
	std::cout << "[DEBUG]genPoint" << std::endl;
	vector3d Bv(1, 0, 0);// ��Ϊ��ֱ��ƽ�������
	if (Planar) {
		sweptEntity.getSweepSurfacePlanar(T, Suv, Bv, S, 10);
	}
	else {
		sweptEntity.getSweepSurface3(T, Suv, Bv, S, 10);// �õ������������ɵ�һ������Ľṹ��
	}
	sweptEntity.outputSweptVolume(cfg.p0, cfg.p1, cfg.p2, cfg.U0, cfg.U1, cfg.U2, cfg.ctrlPoints);//�������ṹ�壬Ȼ�����ƽ��
	cfg.genPointNumX = 20;
	cfg.genPointNumY = 20;
	cfg.genPointNumZ = 20;
	cfg.n2 = cfg.ctrlPoints.size() - 1;
	cfg.n1 = cfg.ctrlPoints[0].size() - 1;
	cfg.n0 = cfg.ctrlPoints[0][0].size() - 1;
	int count = 0;
	for (int i = 0; i < cfg.genPointNumX; i++) {
		std::vector<std::vector<Point3d>> surfacePoints;
		for (int j = 0; j < cfg.genPointNumY; j++) {
			std::vector<Point3d> linePoints;
			for (int k = 0; k < cfg.genPointNumZ; k++) {
				double u = i * 1.0 * (cfg.U0[cfg.U0.size() - 1] - cfg.U0[0]) / cfg.genPointNumX;//���ȷֳɶ��ٸ��������еĽڵ���
				double v = j * 1.0 * (cfg.U1[cfg.U1.size() - 1] - cfg.U1[0]) / cfg.genPointNumY;//���ȷֳɶ��ٸ��������еĽڵ���
				double w = k * 1.0 * (cfg.U2[cfg.U2.size() - 1] - cfg.U2[0]) / cfg.genPointNumZ;//���ȷֳɶ��ٸ��������еĽڵ���
				Point3d V;
				//std::cout << "[DEBUG] ��ʼ��� " << count++ <<std::endl;
				volumePoint(cfg.n0, cfg.p0, cfg.U0,
					cfg.n1, cfg.p1, cfg.U1,
					cfg.n2, cfg.p2, cfg.U2,
					cfg.ctrlPoints, u, v, w, V);// n2 �ǳ���
				//std::cout << "[DEBUG] �м����" << std::endl;
				linePoints.push_back(V);
			}
			surfacePoints.push_back(linePoints);
		}
		cfg.volumePoints.push_back(surfacePoints);
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
void SweptVolumeTransformFrentFrameScale::curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u, Point3d &C) {
	int span = findSpan(n, p, u, U);
	std::vector<double> base;
	basisFuns(span, u, p, U, base);
	double x, y, z;
	x = y = z = 0;
	for (int i = 0; i <= p; i++) {
		x = x + P[span - p + i].x * base[i];
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
@brief   : �������ϵĵ�  ��ʽ��
@author  : lee
@input   ��
		 {
			n0
			p0
			U0
		 }
		 {
			n1
			p1
			U1
		 }
		 {
			n2
			p2
			U2
		 }
		 P ���Ƶ�
		 x
		 y
		 z
@output  ��V
@time    : none
@time    : none
**************************************************/
void SweptVolumeTransformFrentFrameScale::volumePoint(int n0, int p0, const std::vector<double>&U0, int n1, int p1, const std::vector<double> &U1,
	int n2, int p2, const std::vector<double> &U2, std::vector<std::vector<std::vector<Point3d>>>&P,
	double x, double y, double z, Point3d &V) {
	int i0 = findSpan(n0, p0, x, U0);
	int i1 = findSpan(n1, p1, y, U1);
	int i2 = findSpan(n2, p2, z, U2);
	std::vector<double> N0, N1, N2;
	basisFuns(i0, x, p0, U0, N0);
	basisFuns(i1, y, p1, U1, N1);
	basisFuns(i2, z, p2, U2, N2);
	//int uind = uspan - p;
	V.x = V.y = V.z = 0;
	int xind = i0 - p0;
	for (int i = 0; i <= p2; i++) {
		Point3d S(0, 0, 0);
		int zind = i2 - p2 + i;
		for (int j = 0; j <= p1; j++) {
			int yind = i1 - p1 + j;
			Point3d temp(0, 0, 0);
			for (int k = 0; k <= p0; k++) {
				temp.x = temp.x + N0[k] * P[zind][yind][xind + k].x;
				temp.y = temp.y + N0[k] * P[zind][yind][xind + k].y;
				temp.z = temp.z + N0[k] * P[zind][yind][xind + k].z;
			}
			S.x = S.x + N1[j] * temp.x;
			S.y = S.y + N1[j] * temp.y;
			S.z = S.z + N1[j] * temp.z;
		}
		V.x = V.x + N2[i] * S.x;
		V.y = V.y + N2[i] * S.y;
		V.z = V.z + N2[i] * S.z;
	}
}



/**************************************************
@brief   : ���ļ��ж�ȡ��Ӧ�Ĳ���
@author  : lee
@input   ���ļ���(����·��)/��� �������
@output  ��none
@time    : none
**************************************************/
void SweptVolumeTransformFrentFrameScale::jsonReader(std::string fileName) {
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
		std::string name = root["name"].asString();
		Json::Value scPointsJ = root["surfaceCtrlPoints"];
		std::vector<std::vector<Point3d>> scPoint;
		for (int i = 0; i < scPointsJ.size(); i++) {
			std::vector<Point3d> line;
			for (int j = 0; j < scPointsJ[i].size(); j++) {
				Point3d point;
				point.x = scPointsJ[i][j][0].asDouble();
				point.y = scPointsJ[i][j][1].asDouble();
				point.z = scPointsJ[i][j][2].asDouble();
				line.push_back(point);
			}
			scPoint.push_back(line);
		}
		int SP = root["SP_Power"].asInt();
		int SQ = root["SQ_Power"].asInt();
		Json::Value SUJ = root["SU_vector"];
		std::vector<double> SU;
		for (int i = 0; i < SUJ.size(); i++) {
			SU.push_back(SUJ[i].asDouble());
		}
		Json::Value SVJ = root["SV_vector"];
		std::vector<double> SV;
		for (int i = 0; i < SVJ.size(); i++) {
			SV.push_back(SVJ[i].asDouble());
		}
		// ---- �켣���� ----
		Json::Value ccPointsJ = root["curveCtrlPoints"];
		double curvaturePointsNumber = 100;
		std::vector<Point3d> ccPoint;
		for (int i = 0; i < ccPointsJ.size(); i++) {
			Point3d point;
			point.x = ccPointsJ[i][0].asDouble();
			point.y = ccPointsJ[i][1].asDouble();
			point.z = ccPointsJ[i][2].asDouble();
			ccPoint.push_back(point);
		}
		Json::Value CUJ = root["CU_vector"];
		std::vector<double> CU;
		for (int i = 0; i < CUJ.size(); i++) {
			CU.push_back(CUJ[i].asDouble());
		}
		int CP = root["CP_Power"].asInt();

		
		// ʹ�� �ṹ���ʼ���������ߵĽṹ�� (int p_, int n_, std::vector<double> U_, std::vector<Point3d> controlPts_) :
		BSplineCurveAbstract bcu(CP, ccPoint.size() - 1, CU, ccPoint);
		T.set(bcu);
		// move the section curve to origin // QU:ΪʲôҪ��section Curve �ƶ���ԭ��
		Point3d translation(scPoint[0][0]);
		for (int i = 0; i < scPoint.size(); i++) {
			for (int j = 0; j < scPoint[i].size(); j++) {
				scPoint[i][j] = scPoint[i][j] - translation;
			}
		}
		BSplineSurfaceAbstract SF(SP, scPoint.size() - 1, SQ, scPoint[0].size(),
			SU, SV, scPoint);
		Suv.set(SF);

		curvePoints.clear();
		for (int i = 0; i <= curvaturePointsNumber; i++) {
			double u = i * 1.0 * (CU[CU.size() - 1] - CU[0]) / curvaturePointsNumber;//���ȷֳɶ��ٸ��������еĽڵ���
			Point3d C;
			curvePoint(ccPoint.size() - 1, CP, CU, ccPoint, u, C);
			curvePoints.push_back(C);
		}
		// ---- scale���� ----
		Json::Value vectorsScale = root["scaleCtrlPoints"];
		std::vector<Point3d> scalecPoint;
		for (int i = 0; i < vectorsScale.size(); i++) {
			Point3d point;
			point.x = vectorsScale[i][0].asDouble();
			point.y = vectorsScale[i][1].asDouble();
			point.z = vectorsScale[i][2].asDouble();
			scalecPoint.push_back(point);
		}
		Json::Value ScaleCurveUJ = root["ScaleU_vector"];
		std::vector<double> scalecU;
		for (int i = 0; i < ScaleCurveUJ.size(); i++) {
			scalecU.push_back(ScaleCurveUJ[i].asDouble());
		}
		int scaleCurvePower = root["ScaleP_Power"].asInt();
		BSplineCurveAbstract temp1(scaleCurvePower, scalecPoint.size() - 1, scalecU, scalecPoint);
		S.set(temp1);

		// ---- Planar ----
		Planar = root["Planar"].asBool();
	}

}


/**************************************************
@brief   : ��ʼ��opengl����
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/
void SweptVolumeTransformFrentFrameScale::initializeGL() {
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
void SweptVolumeTransformFrentFrameScale::resizeGL(int width, int height)
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
void SweptVolumeTransformFrentFrameScale::paintGL() {
	//genPoints("normal");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// �����Ļ����Ȼ���
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//ƽ��

	glRotated(rotationX, 1.0, 0.0, 0.0);
	glRotated(rotationY, 0.0, 1.0, 0.0);
	glRotated(rotationZ, 0.0, 0.0, 1.0);//��ת
	glScalef(times, times, times);//����
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
	// �������Ƶ�
	glPointSize(8.f);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);
	for (int i = 0; i < cfg.ctrlPoints.size(); i++) {
		for (int j = 0; j < cfg.ctrlPoints[i].size(); j++) {
			for (int k = 0; k < cfg.ctrlPoints[i][j].size(); k++) {
				glVertex3f(cfg.ctrlPoints[i][j][k].x, cfg.ctrlPoints[i][j][k].y,
					cfg.ctrlPoints[i][j][k].z);
			}
		}
	}
	glEnd();
	// ��������
	for (int t = 0; t < cfg.volumePoints.size(); t++) {
		for (int i = 0; i < cfg.volumePoints[t].size(); i++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int j = 0; j < cfg.volumePoints[t][0].size(); j++) {
				glVertex3d(cfg.volumePoints[t][i][j].x, cfg.volumePoints[t][i][j].y, cfg.volumePoints[t][i][j].z);
			}
			glEnd();
		}
	}
	for (int t = 0; t < cfg.volumePoints.size(); t++) {
		for (int i = 0; i < cfg.volumePoints[t][0].size(); i++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int j = 0; j < cfg.volumePoints[t].size(); j++) {
				glVertex3d(cfg.volumePoints[t][j][i].x, cfg.volumePoints[t][j][i].y, cfg.volumePoints[t][j][i].z);
			}
			glEnd();
		}
	}
	// ������ֱ������
	for (int i = 0; i < cfg.volumePoints[0].size(); i++) {
		for (int j = 0; j < cfg.volumePoints[0][0].size(); j++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int t = 0; t < cfg.volumePoints.size(); t++) {
				glVertex3d(cfg.volumePoints[t][i][j].x, cfg.volumePoints[t][i][j].y, cfg.volumePoints[t][i][j].z);
			}
			glEnd();
		}
	}
	//���� �켣����
	glBegin(GL_LINE_STRIP);
	glColor3f(1.0, 1.0, 0.0);
	for (int i = 0; i < curvePoints.size(); i++) {
		glVertex3f(curvePoints[i].x, curvePoints[i].y, curvePoints[i].z);
	}
	glEnd();
	//for (int t = 0; t < cfg.volumePoints.size(); t++) {
	//	for (int i = 0; i < cfg.volumePoints[t].size(); i++) {
	//		glBegin(GL_LINE_STRIP);
	//		glColor3f(1.0, 1.0, 1.0);
	//		for (int j = 0; j < cfg.volumePoints[t][0].size(); j++) {
	//			if (j == 0 || i == 0 || t == 0 || i == cfg.volumePoints[t].size() - 1 || j == cfg.volumePoints[t][0].size() - 1 || t == cfg.volumePoints.size() - 1)
	//				glVertex3d(cfg.volumePoints[t][i][j].x, cfg.volumePoints[t][i][j].y, cfg.volumePoints[t][i][j].z);
	//		}
	//		glEnd();
	//	}
	//}
	//for (int t = 0; t < cfg.volumePoints.size(); t++) {
	//	for (int i = 0; i < cfg.volumePoints[t][0].size(); i++) {
	//		glBegin(GL_LINE_STRIP);
	//		glColor3f(1.0, 1.0, 1.0);
	//		for (int j = 0; j < cfg.volumePoints[t].size(); j++) {
	//			if (j == 0 || i == 0 || t == 0 || i == cfg.volumePoints[t][0].size() - 1 || j == cfg.volumePoints[t].size() - 1 || t == cfg.volumePoints.size() - 1)
	//				glVertex3d(cfg.volumePoints[t][j][i].x, cfg.volumePoints[t][j][i].y, cfg.volumePoints[t][j][i].z);
	//		}
	//		glEnd();
	//	}
	//}
	//for (int i = 0; i < cfg.volumePoints[0].size(); i++) {
	//	for (int j = 0; j < cfg.volumePoints[0][0].size(); j++) {
	//		glBegin(GL_LINE_STRIP);
	//		glColor3f(1.0, 1.0, 1.0);
	//		for (int t = 0; t < cfg.volumePoints.size(); t++) {
	//			if (j == 0 || i == 0 || t == 0 || i == cfg.volumePoints[0].size() - 1 || j == cfg.volumePoints[0][0].size() - 1 || t == cfg.volumePoints.size() - 1)
	//				glVertex3d(cfg.volumePoints[t][i][j].x, cfg.volumePoints[t][i][j].y, cfg.volumePoints[t][i][j].z);
	//		}
	//		glEnd();
	//	}
	//}
}



/**************************************************
@brief   : ���������¼�
@author  : lee
@input   ��e  �¼�
@output  ��none
@time    : none
**************************************************/
void SweptVolumeTransformFrentFrameScale::mousePressEvent(QMouseEvent *e) {
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
void SweptVolumeTransformFrentFrameScale::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : �������¼�
@author  : lee
@input   ��event
@output  ��none
@time    : none
**************************************************/
void SweptVolumeTransformFrentFrameScale::wheelEvent(QWheelEvent *event) {
	if (event->delta() > 0) {// ������Զ��ʹ����ʱ
		times += 0.008f;
		update();
	}
	else {//��������ʹ���߷�����תʱ
		times -= 0.008f;
		update();
	}
}



/**************************************************
@brief   : none
@author  : none
@input   ��none
@output  ��none
@time    : none
**************************************************/
void SweptVolumeTransformFrentFrameScale::mouseMoveEvent(QMouseEvent *e) {
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


/**************************************************
@brief   : ����obj�ļ�
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/
void SweptVolumeTransformFrentFrameScale::exportOBJ() {
	MyMesh mesh;
	int z = cfg.volumePoints.size();
	int x = cfg.volumePoints[0].size();
	int y = cfg.volumePoints[0][0].size();
	int faceDir[][2] = { {0, z - 1},{0, x - 1},{0, y - 1} };

	for (int t = 0; t < 1; t++) {
		std::vector<std::vector<Point3d>> v;//����
		for (int i = 0; i < x; i++) {
			std::vector<Point3d> tmp;
			for (int j = 0; j < y; j++) {
				Point3d p(cfg.volumePoints[faceDir[0][t]][i][j].x, cfg.volumePoints[faceDir[0][t]][i][j].y, cfg.volumePoints[faceDir[0][t]][i][j].z);
				tmp.push_back(p);
			}
			v.push_back(tmp);
		}
		genOneFaceMesh(mesh, v.size(), v[0].size(), v);
	}
	for (int t = 1; t < 2; t++) {
		std::vector<std::vector<Point3d>> v;//����
		for (int i = 0; i < x; i++) {
			std::vector<Point3d> tmp;
			for (int j = 0; j < y; j++) {
				Point3d p(cfg.volumePoints[faceDir[0][t]][i][j].x, cfg.volumePoints[faceDir[0][t]][i][j].y, cfg.volumePoints[faceDir[0][t]][i][j].z);
				tmp.push_back(p);
			}
			v.push_back(tmp);
		}
		genOtherFaceMesh(mesh, v.size(), v[0].size(), v);
	}
	for (int t = 1; t < 2; t++) {
		std::vector<std::vector<Point3d>> v;//����1
		for (int i = 0; i < x; i++) {
			std::vector<Point3d> tmp;
			for (int j = 0; j < z; j++) {
				Point3d p(cfg.volumePoints[j][i][faceDir[2][t]].x, cfg.volumePoints[j][i][faceDir[2][t]].y, cfg.volumePoints[j][i][faceDir[2][t]].z);
				tmp.push_back(p);
			}
			v.push_back(tmp);
		}
		genOneFaceMesh(mesh, v.size(), v[0].size(), v);
	}
	for (int t = 0; t < 1; t++) {
		std::vector<std::vector<Point3d>> v;//����1
		for (int i = 0; i < x; i++) {
			std::vector<Point3d> tmp;
			for (int j = 0; j < z; j++) {
				Point3d p(cfg.volumePoints[j][i][faceDir[2][t]].x, cfg.volumePoints[j][i][faceDir[2][t]].y, cfg.volumePoints[j][i][faceDir[2][t]].z);
				tmp.push_back(p);
			}
			v.push_back(tmp);
		}
		genOtherFaceMesh(mesh, v.size(), v[0].size(), v);
	}
	for (int t = 0; t < 1; t++) {
		std::vector<std::vector<Point3d>> v;//����2
		for (int i = 0; i < y; i++) {
			std::vector<Point3d> tmp;
			for (int j = 0; j < z; j++) {
				Point3d p(cfg.volumePoints[j][faceDir[1][t]][i].x, cfg.volumePoints[j][faceDir[1][t]][i].y, cfg.volumePoints[j][faceDir[1][t]][i].z);
				tmp.push_back(p);
			}
			v.push_back(tmp);
		}
		genOneFaceMesh(mesh, v.size(), v[0].size(), v);
	}
	for (int t = 1; t < 2; t++) {
		std::vector<std::vector<Point3d>> v;//����2
		for (int i = 0; i < y; i++) {
			std::vector<Point3d> tmp;
			for (int j = 0; j < z; j++) {
				Point3d p(cfg.volumePoints[j][faceDir[1][t]][i].x, cfg.volumePoints[j][faceDir[1][t]][i].y, cfg.volumePoints[j][faceDir[1][t]][i].z);
				tmp.push_back(p);
			}
			v.push_back(tmp);
		}
		genOtherFaceMesh(mesh, v.size(), v[0].size(), v);
	}

	// write mesh to output.obj
	try
	{
		if (!OpenMesh::IO::write_mesh(mesh, "output.off"))
		{
			std::cerr << "Cannot write mesh to file 'output8.off'" << std::endl;
		}
	}
	catch (std::exception& x)
	{
		std::cerr << x.what() << std::endl;
	}
	std::cout << "������" << std::endl;
}



/**************************************************
@brief   : ����һ�����mesh
@author  : lee
@input   ��mesh ��������
		   m ��
		   n ��
		   v ��ά���������
@output  ��none
@time    : none
**************************************************/
void SweptVolumeTransformFrentFrameScale::genOneFaceMesh(MyMesh &mesh, int m, int n, const std::vector<std::vector<Point3d>> &v) {
	MyMesh::VertexHandle **vhandle = new MyMesh::VertexHandle*[m];
	for (int i = 0; i < m; i++) {
		vhandle[i] = new MyMesh::VertexHandle[n];//��ά������
	}
	std::vector<MyMesh::VertexHandle>face_vhandles;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			vhandle[i][j] = mesh.add_vertex(MyMesh::Point(v[i][j].x, v[i][j].y, v[i][j].z));
		}
	}
	for (int i = 0; i < m - 1; i++) {
		for (int j = 0; j < n - 1; j++) {
			face_vhandles.clear();
			face_vhandles.push_back(vhandle[i + 1][j]);//����������
			face_vhandles.push_back(vhandle[i][j + 1]);
			face_vhandles.push_back(vhandle[i][j]);

			mesh.add_face(face_vhandles);
			face_vhandles.clear();
			face_vhandles.push_back(vhandle[i + 1][j]);
			face_vhandles.push_back(vhandle[i + 1][j + 1]);
			face_vhandles.push_back(vhandle[i][j + 1]);
			mesh.add_face(face_vhandles);
		}
	}

}


/**************************************************
@brief   : �������෴��
@author  : lee
@input   : mesh ��������
		   m ��
		   n ��
		   v ��ά���������
@output  ��none
@time    : none
**************************************************/
void SweptVolumeTransformFrentFrameScale::genOtherFaceMesh(MyMesh &mesh, int m, int n, const std::vector<std::vector<Point3d>> &v) {
	MyMesh::VertexHandle **vhandle = new MyMesh::VertexHandle*[m];
	for (int i = 0; i < m; i++) {
		vhandle[i] = new MyMesh::VertexHandle[n];//��ά������
	}
	std::vector<MyMesh::VertexHandle>face_vhandles;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			vhandle[i][j] = mesh.add_vertex(MyMesh::Point(v[i][j].x, v[i][j].y, v[i][j].z));
		}
	}
	for (int i = 0; i < m - 1; i++) {
		for (int j = 0; j < n - 1; j++) {
			face_vhandles.clear();
			face_vhandles.push_back(vhandle[i][j + 1]);
			face_vhandles.push_back(vhandle[i + 1][j]);//����������
			face_vhandles.push_back(vhandle[i][j]);

			mesh.add_face(face_vhandles);
			face_vhandles.clear();
			face_vhandles.push_back(vhandle[i + 1][j + 1]);
			face_vhandles.push_back(vhandle[i + 1][j]);
			face_vhandles.push_back(vhandle[i][j + 1]);
			mesh.add_face(face_vhandles);
		}
	}

}


