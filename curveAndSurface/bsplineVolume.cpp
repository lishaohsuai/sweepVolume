#include "bsplineVolume.h"
#include <algorithm>


//�ο� The NURBS Book 2nd (38/660)
BSplineVolume::BSplineVolume(QWidget *parent)//����ȷ�����ƶ�����Ŀ�Ϳ��ƶ���
	:Strategy(), QGLWidget(parent) {
	rotationX = 0;
	rotationY = 0;
	rotationZ = 0;
	flag = -1;
	times = 1.0;
}


/**************************************************
@brief   : ���������ͷ������Դ
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/
BSplineVolume::~BSplineVolume() {}


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
//int BSplineVolume::findSpan(int n, int p, double u, const std::vector<double> &U) {
//	// �ܾ������������ ��������ѭ��
//	if (u == U[n + 1]) return n;
//	int low = p;
//	int high = n + 1;
//	int mid = (low + high) / 2;
//	while (u < U[mid] || u >= U[mid + 1]) {
//		if (u < U[mid]) high = mid;
//		else low = mid;
//		mid = (low + high) / 2.0 + 0.5;
//	}
//	return mid;
//}
int BSplineVolume::findSpan(int n, int p, double u, const std::vector<double> &U) {
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
void BSplineVolume::basisFuns(int i, double u, int p, const std::vector<double> &U, std::vector<double> &N) {
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
**************************************************/
//void BSplineVolume::volumePoint(int n0, int p0, const std::vector<double>&U0, int n1, int p1, const std::vector<double> &U1,
//	int n2, int p2, const std::vector<double> &U2, std::vector<std::vector<std::vector<Point3d>>>&P, 
//	double x, double y, double z, Point3d &V) {
//	// �ƺ����������
//	int i0 = findSpan(n0, p0, x, U0);
//	int i1 = findSpan(n1, p1, y, U1);
//	int i2 = findSpan(n2, p2, z, U2);
//	std::vector<double> N0, N1, N2;
//	basisFuns(i0, x, p0, U0, N0);
//	basisFuns(i1, y, p1, U1, N1);
//	basisFuns(i2, z, p2, U2, N2);
//	Point3d temp(0, 0, 0);
//	for (int j0 = i0 - bsplineVCfg.p0; j0 <= i0; ++j0) {
//		for (int j1 = i1 - bsplineVCfg.p1; j1 <= i1; ++j1) {
//			for (int j2 = i2 - bsplineVCfg.p2; j2 <= i2; ++j2) {
//				//std::cout << "[DEBUG] j0 j1 j2 " << j0 << " " << j1 << " " << j2 << std::endl;
//				//std::cout << "[DEBUG] i0 i1 i2 " << i0 << " " << i1 << " " << i2 << std::endl;
//				//std::cout << "[DEBUG] N0 N1 N2 " << N0.size() << " " << N1.size() << " " << N2.size() << " Psize " << P.size() << std::endl;
//				temp.x += N0[j0] * N1[j1] * N2[j2] * P[j0][j1][j2].x;
//				temp.y += N0[j0] * N1[j1] * N2[j2] * P[j0][j1][j2].y;
//				temp.z += N0[j0] * N1[j1] * N2[j2] * P[j0][j1][j2].z;
//			}
//		}
//	}
//	V.x = temp.x;
//	V.y = temp.y;
//	V.z = temp.z;
//}
void BSplineVolume::volumePoint(int n0, int p0, const std::vector<double>&U0, int n1, int p1, const std::vector<double> &U1,
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
@brief   : ʹ�ù�ʽ������BSpline Volume�ĵ�
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/
void BSplineVolume::genPoints(std::string method) {
	jsonReader();
	bsplineVCfg.volumePoints.clear();
	for (int i = 0; i <= bsplineVCfg.genPointNumX; i++) {
		std::vector<std::vector<Point3d>> surfacePoints;
		for (int j = 0; j <= bsplineVCfg.genPointNumY; j++) {
			std::vector<Point3d> linePoints;
			for (int k = 0; k <= bsplineVCfg.genPointNumZ; k++) {
				double u = (double)i / (double)bsplineVCfg.genPointNumX;
				double v = (double)j / (double)bsplineVCfg.genPointNumY;
				double w = (double)k / (double)bsplineVCfg.genPointNumZ;
				Point3d genPoint;
				volumePoint(bsplineVCfg.n0, bsplineVCfg.p0, bsplineVCfg.U0,
					bsplineVCfg.n1, bsplineVCfg.p1, bsplineVCfg.U1,
					bsplineVCfg.n2, bsplineVCfg.p2, bsplineVCfg.U2,
					bsplineVCfg.ctrlPoints, u, v, w, genPoint);
				linePoints.push_back(genPoint);
			}
			surfacePoints.push_back(linePoints);
		}
		bsplineVCfg.volumePoints.push_back(surfacePoints);
	}
}

void BSplineVolume::initializeGL()
{
	// ����smooth shading(��Ӱƽ��)
	glShadeModel(GL_SMOOTH);
	// ���������Ļ���õ���ɫ ɫ��ֵ��1Χ��0.0-1.0 (R G B A) ����ʱA��������
	glClearColor(0.1, 0.1, 0.4, 1.0);
	/* �������б�����,�ǹ�����Ȼ����,��Ȼ�����OpenGLʮ����Ҫ�Ĳ���*/
	// ������Ȼ���
	glClearDepth(1.0);
	// ������Ȳ���
	glEnable(GL_DEPTH_TEST);
	// ������Ȳ��Ե�����
	glDepthFunc(GL_LEQUAL);
	// ����ϣ���õ���õ�͸������
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

void BSplineVolume::resizeGL(int width, int height)
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
// (X������������ң�Y������������ϣ�Z�������������)
// OpenGL��Ļ���ĵ�����ֵ��X��Y���ϵ�0.0��
// �������������ֵ�Ǹ�ֵ����������ֵ��
// ������Ļ��������ֵ��������Ļ�׶��Ǹ�ֵ��
// ������Ļ��Ǹ�ֵ���Ƴ���Ļ������ֵ��
void BSplineVolume::paintGL() {

	//std::cout << "[DEBUG] BSplineVolume paintEvent" << std::endl;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// �����Ļ����Ȼ���
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//ƽ��
	glRotatef(rotationX, 1.0, 0.0, 0.0);
	glRotatef(rotationY, 0.0, 1.0, 0.0);
	glRotatef(rotationZ, 0.0, 0.0, 1.0);//��ת
	glScalef(times, times, times);//����
	// �������Ƶ�
	glPointSize(8.f);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);
	for (int i = 0; i < bsplineVCfg.ctrlPoints.size(); i++) {
		for (int j = 0; j < bsplineVCfg.ctrlPoints[i].size(); j++) {
			for (int k = 0; k < bsplineVCfg.ctrlPoints[i][j].size(); k++) {
				glVertex3f(bsplineVCfg.ctrlPoints[i][j][k].x, bsplineVCfg.ctrlPoints[i][j][k].y,
					bsplineVCfg.ctrlPoints[i][j][k].z);
			}
		}
	}
	glEnd();
	// ��������
	for (int t = 0; t < bsplineVCfg.volumePoints.size(); t++) {
		for (int i = 0; i < bsplineVCfg.volumePoints[t].size(); i++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int j = 0; j < bsplineVCfg.volumePoints[t][0].size(); j++) {
				if (j == 0 || i == 0 || t == 0 || i == bsplineVCfg.volumePoints[t].size() - 1 || j == bsplineVCfg.volumePoints[t][0].size() - 1 || t == bsplineVCfg.volumePoints.size() - 1)
					glVertex3d(bsplineVCfg.volumePoints[t][i][j].x, bsplineVCfg.volumePoints[t][i][j].y, bsplineVCfg.volumePoints[t][i][j].z);
			}
			glEnd();
		}
	}
	for (int t = 0; t < bsplineVCfg.volumePoints.size(); t++) {
		for (int i = 0; i < bsplineVCfg.volumePoints[t][0].size(); i++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int j = 0; j < bsplineVCfg.volumePoints[t].size(); j++) {
				if (j == 0 || i == 0 || t == 0 || i == bsplineVCfg.volumePoints[t][0].size() - 1 || j == bsplineVCfg.volumePoints[t].size() - 1 || t == bsplineVCfg.volumePoints.size() - 1)
					glVertex3d(bsplineVCfg.volumePoints[t][j][i].x, bsplineVCfg.volumePoints[t][j][i].y, bsplineVCfg.volumePoints[t][j][i].z);
			}
			glEnd();
		}
	}
	for (int i = 0; i < bsplineVCfg.volumePoints[0].size(); i++) {
		for (int j = 0; j < bsplineVCfg.volumePoints[0][0].size(); j++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int t = 0; t < bsplineVCfg.volumePoints.size(); t++) {
				if (j == 0 || i == 0 || t == 0 || i == bsplineVCfg.volumePoints[0].size() - 1 || j == bsplineVCfg.volumePoints[0][0].size() - 1 || t == bsplineVCfg.volumePoints.size() - 1)
					glVertex3d(bsplineVCfg.volumePoints[t][i][j].x, bsplineVCfg.volumePoints[t][i][j].y, bsplineVCfg.volumePoints[t][i][j].z);
			}
			glEnd();
		}
	}
}

void BSplineVolume::jsonReader() {
	QString QfileName = QFileDialog::getOpenFileName(NULL,
		QObject::tr("Open json file"),
		QObject::tr("./config"),
		QObject::tr("mesh (*.json);;"
			"All Files (*)"));
	std::string fileName = QfileName.toStdString();
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
		bsplineVCfg.genPointNumX = root["genPointNumX"].asInt();
		bsplineVCfg.genPointNumY = root["genPointNumY"].asInt();
		bsplineVCfg.genPointNumZ = root["genPointNumZ"].asInt();
		bsplineVCfg.name = root["name"].asString();
		Json::Value points = root["Point"];
		struct Point3d p(0, 0, 0);
		for (int i = 0; i < points.size(); i++) {
			std::vector<std::vector<Point3d>> surface;
			for (int j = 0; j < points[i].size(); j++) {
				std::vector<struct Point3d> line;
				for (int k = 0; k < points[i][j].size(); k++) {
					Point3d p;
					p.x = points[i][j][k][0].asDouble();
					p.y = points[i][j][k][1].asDouble();
					p.z = points[i][j][k][2].asDouble();
					line.push_back(p);
				}
				surface.push_back(line);
			}
			bsplineVCfg.ctrlPoints.push_back(surface);
		}
		bsplineVCfg.n0 = bsplineVCfg.ctrlPoints.size() - 1;//���ǵ�ĸ�����ȥ1
		bsplineVCfg.n1 = bsplineVCfg.ctrlPoints[0].size() - 1;
		bsplineVCfg.n2 = bsplineVCfg.ctrlPoints[0][0].size() - 1;
		bsplineVCfg.p0 = root["P0"].asInt();
		bsplineVCfg.p1 = root["P1"].asInt();
		bsplineVCfg.p2 = root["P1"].asInt();
		Json::Value U0 = root["U0"];
		for (int i = 0; i < U0.size(); i++) {
			bsplineVCfg.U0.push_back(U0[i].asDouble());
		}
		Json::Value U1 = root["U1"];
		for (int i = 0; i < U1.size(); i++) {
			bsplineVCfg.U1.push_back(U1[i].asDouble());
		}
		Json::Value U2 = root["U2"];
		for (int i = 0; i < U2.size(); i++) {
			bsplineVCfg.U2.push_back(U2[i].asDouble());
		}
	}
}


Point3d BSplineVolume::changeto3d(double x, double y)
{
	GLint		viewport[4] = { 0 };
	GLdouble	modelview[16] = { 0 };
	GLdouble	projection[16] = { 0 };
	GLfloat		winX = 0.0f;
	GLfloat		winY = 0.0f;
	GLfloat		winZ = 0.0f;
	GLdouble	posX = 0.0f;
	GLdouble	posY = 0.0f;
	GLdouble	posZ = 0.0f;
	glGetIntegerv(GL_VIEWPORT, viewport); // ��ȡ��ͼ����
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);//��ȡģ����ͼ����
	glGetDoublev(GL_PROJECTION_MATRIX, projection);//��ȡͶӰ����
	glPopMatrix();//��ջ���ָ��ո���ջ�ľ�����Ϣ
	winX = x;
	winY = width() - y;    //widthΪ��Ļ�Ŀ�ȣ���Qt����Ҫ����Ϊheight���ȸ߶ȣ�   
	//������ͨ����ȡ���صķ�ʽ���z����
	glReadPixels((int)winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);//��ȡ��Ļ���  
	//����ת��
	gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
	Point3d pos = { posX,posY,posZ };
	return pos;
}

/*
//������2���϶����ƶ���
void BSplineVolume::mousePressEvent(QMouseEvent *event)
{
	Point3d p1 = changeto3d(event->pos().x(), event->pos().y());
	for (int i = 0; i <= bsplineVCfg.n; i++)
		for (int j = 0; j <= bsplineVCfg.m; j++)
		{
			Point3d p2 = bsplineVCfg.ctrlPoints[i][j];
			if (((p1.x >= p2.x - 4) && (p1.x <= p2.x + 4)) && ((p1.y >= p2.y - 4) && (p1.y <= p2.y + 4)) && ((p1.z >= p2.z - 4) && (p1.z <= p2.z + 4)))
			{
				flag = 1;
				ii = i;
				jj = j;
			}
		}
}

void BSplineVolume::mouseMoveEvent(QMouseEvent *event){
	if (flag == 1){
		Point3d p = changeto3d(event->pos().x(), event->pos().y());
		bsplineVCfg.ctrlPoints[ii][jj] = p;
		updateGL();
	}
}
*/

void BSplineVolume::mouseReleaseEvent(QMouseEvent *event) {
	flag = -1;
}


//������1���۲�����
void BSplineVolume::mousePressEvent(QMouseEvent *event) {
	lastPos = event->pos();
	flag = 1;
}



/**************************************************
@brief   : �������¼�
@author  : lee
@input   ��event
@output  ��none
@time    : none
**************************************************/
void BSplineVolume::wheelEvent(QWheelEvent *event) {
	if (event->delta() > 0) {// ������Զ��ʹ����ʱ
		times += 0.008f;
		update();
	}
	else {//��������ʹ���߷�����תʱ
		times -=0.008f;
		update();
	}
}

void BSplineVolume::mouseMoveEvent(QMouseEvent *event)
{
	if (flag == 1)
	{
		//Ϊʲô��ô���㣬��̫�˽⣬�ȷ���
		GLfloat dx = GLfloat(event->x() - lastPos.x()) / width();
		GLfloat dy = GLfloat(event->y() - lastPos.y()) / height();
		if (event->buttons() & Qt::LeftButton) {
			rotationX += 180 * dy;
			rotationY += 180 * dx;
			updateGL();//ÿ�ζ�Ҫ���¼���㣬��Ӧ��Щ��
		}
		else if (event->buttons() & Qt::RightButton) {
			rotationX += 180 * dy;
			rotationZ += 180 * dx;
			updateGL();
		}
		lastPos = event->pos();
	}
}
