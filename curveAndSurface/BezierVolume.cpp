#include "BezierVolume.h"
#include <algorithm>


//�ο� The NURBS Book 2nd (38/660)
BezierVolume::BezierVolume(QWidget *parent)//����ȷ�����ƶ�����Ŀ�Ϳ��ƶ���
	:Strategy(), QGLWidget(parent) {
	rotationX = 0;
	rotationY = 0;
	rotationZ = 0;
	flag = -1;
}

BezierVolume::~BezierVolume() {}


//����ƽ���϶�Ӧ�ĵ�
void BezierVolume::calcDeCasteljau2(const std::vector<std::vector<Point3d> > &P, int n, int m, double u0
	, double v0, Point3d &S) {
	std::vector<Point3d> Q;
	Q.resize(m + 1);
	for (int j = 0; j <= m; j++) {
		calcDeCasteljau1(P[j], n, u0, Q[j]);
	}
	calcDeCasteljau1(Q, m, v0, S);
}

void BezierVolume::calcDeCasteljau1(const std::vector<Point3d> &P, int n, double u, Point3d & _Q) {
	std::vector<Point3d> Q;
	for (int i = 0; i <= n; i++) {
		Q.push_back(P[i]);
	}
	for (int k = 1; k <= n; k++) {
		for (int i = 0; i <= n - k; i++) {
			Q[i].x = (1.0 - u) * Q[i].x + u * Q[i + 1].x;
			Q[i].y = (1.0 - u) * Q[i].y + u * Q[i + 1].y;
			Q[i].z = (1.0 - u) * Q[i].z + u * Q[i + 1].z;
		}
	}
	_Q = Q[0];
}


/**************************************************
@brief   : ����ײ�
@author  : lee
@input   : u ��0��1����
		   n ��ʾ����
		   i ��ʾ��������ǵڼ���
		   factorialN ��ʾ �׳� ��ʽ�ļ���
@output  ��none
@time    : none
**************************************************/
double BezierVolume::calcBernstein(double u, int n, int i, std::vector<int> &factorialN) {
	return factorialN[n] * pow(u, i)*pow((1 - u), n - i) / (double)(factorialN[i] * factorialN[n - i]);
}



/**************************************************
@brief   : ����׳�
@author  : lee
@input   ��factorialN ������
		   n  �ܹ��Ľ׳���
@output  ��none
@time    : none
**************************************************/
void BezierVolume::calcfactorialN(std::vector<int> &factorialN, int n) {
	int index = 1;
	for (int i = 0; i <= n; i++) {
		if (i == 0) {}
		else {
			index = index * i;
		}
		factorialN.push_back(index);
	}
}

/**************************************************
@brief   : ʹ�ù�ʽ������Bezier Volume�ĵ�
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/
void BezierVolume::genPoints(std::string method) {
	jsonReader();
	bezierVolumeConfig.volumePoints.clear();
	std::vector<int> factorialN;
	calcfactorialN(factorialN, std::max(std::max(bezierVolumeConfig.n, bezierVolumeConfig.v), bezierVolumeConfig.m));
	for (int i = 0; i <= bezierVolumeConfig.genPointNumX; i++) {
		std::vector<std::vector<Point3d>> surfacePoints;
		for (int j = 0; j <= bezierVolumeConfig.genPointNumY; j++) {
			std::vector<Point3d> linePoints;
			for (int k = 0; k <= bezierVolumeConfig.genPointNumZ; k++) {
				double u = (double)i / (double)bezierVolumeConfig.genPointNumX;
				double v = (double)j / (double)bezierVolumeConfig.genPointNumY;
				double w = (double)k / (double)bezierVolumeConfig.genPointNumZ;
				Point3d genPoint;
				for (int ii = 0; ii <= bezierVolumeConfig.n; ii++) {
					for (int jj = 0; jj <= bezierVolumeConfig.m; jj++) {
						for (int kk = 0; kk <= bezierVolumeConfig.v; kk++) {
							genPoint = genPoint + bezierVolumeConfig.ctrlPoints[ii][jj][kk]
								* calcBernstein(u, bezierVolumeConfig.n, ii, factorialN) 
								* calcBernstein(v, bezierVolumeConfig.m, jj, factorialN) 
								* calcBernstein(w, bezierVolumeConfig.v, kk, factorialN);
						}
					}
				}
				linePoints.push_back(genPoint);
			}
			surfacePoints.push_back(linePoints);
		}
		bezierVolumeConfig.volumePoints.push_back(surfacePoints);
	}
}

void BezierVolume::initializeGL()
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

void BezierVolume::resizeGL(int width, int height)
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
void BezierVolume::paintGL() {

	//std::cout << "[DEBUG] BezierVolume paintEvent" << std::endl;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// �����Ļ����Ȼ���
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//ƽ��
	glRotatef(rotationX, 1.0, 0.0, 0.0);
	glRotatef(rotationY, 0.0, 1.0, 0.0);
	glRotatef(rotationZ, 0.0, 0.0, 1.0);//��ת
	// �������Ƶ�
	glPointSize(8.f);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);
	for (int i = 0; i < bezierVolumeConfig.ctrlPoints.size(); i++){
		for (int j = 0; j < bezierVolumeConfig.ctrlPoints[i].size(); j++) {
			for (int k = 0; k < bezierVolumeConfig.ctrlPoints[i][j].size(); k++) {
				glVertex3f(bezierVolumeConfig.ctrlPoints[i][j][k].x, bezierVolumeConfig.ctrlPoints[i][j][k].y,
					bezierVolumeConfig.ctrlPoints[i][j][k].z);
			}
		}
	}
	glEnd();
	// ��������
	for (int t = 0; t < bezierVolumeConfig.volumePoints.size(); t++) {
		for (int i = 0; i < bezierVolumeConfig.volumePoints[t].size(); i++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int j = 0; j < bezierVolumeConfig.volumePoints[t][0].size(); j++) {
				if (j == 0 || i == 0 || t == 0 || i == bezierVolumeConfig.volumePoints[t].size() - 1 || j == bezierVolumeConfig.volumePoints[t][0].size() - 1 || t == bezierVolumeConfig.volumePoints.size() - 1)
					glVertex3d(bezierVolumeConfig.volumePoints[t][i][j].x, bezierVolumeConfig.volumePoints[t][i][j].y, bezierVolumeConfig.volumePoints[t][i][j].z);
			}
			glEnd();
		}
	}
	for (int t = 0; t < bezierVolumeConfig.volumePoints.size(); t++) {
		for (int i = 0; i < bezierVolumeConfig.volumePoints[t][0].size(); i++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int j = 0; j < bezierVolumeConfig.volumePoints[t].size(); j++) {
				if (j == 0 || i == 0 || t == 0 || i == bezierVolumeConfig.volumePoints[t][0].size() - 1 || j == bezierVolumeConfig.volumePoints[t].size() - 1 || t == bezierVolumeConfig.volumePoints.size() - 1)
					glVertex3d(bezierVolumeConfig.volumePoints[t][j][i].x, bezierVolumeConfig.volumePoints[t][j][i].y, bezierVolumeConfig.volumePoints[t][j][i].z);
			}
			glEnd();
		}
	}
	for (int i = 0; i < bezierVolumeConfig.volumePoints[0].size(); i++) {
		for (int j = 0; j < bezierVolumeConfig.volumePoints[0][0].size(); j++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int t = 0; t < bezierVolumeConfig.volumePoints.size(); t++) {
				if (j == 0 || i == 0 || t == 0 || i == bezierVolumeConfig.volumePoints[0].size() - 1 || j == bezierVolumeConfig.volumePoints[0][0].size() - 1 || t == bezierVolumeConfig.volumePoints.size() - 1)
					glVertex3d(bezierVolumeConfig.volumePoints[t][i][j].x, bezierVolumeConfig.volumePoints[t][i][j].y, bezierVolumeConfig.volumePoints[t][i][j].z);
			}
			glEnd();
		}
	}
}

void BezierVolume::jsonReader() {
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
		bezierVolumeConfig.genPointNumX = root["genPointNumX"].asInt();
		bezierVolumeConfig.genPointNumY = root["genPointNumY"].asInt();
		bezierVolumeConfig.genPointNumZ = root["genPointNumZ"].asInt();
		bezierVolumeConfig.name = root["name"].asString();
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
			bezierVolumeConfig.ctrlPoints.push_back(surface);
		}
		bezierVolumeConfig.n = bezierVolumeConfig.ctrlPoints.size() - 1;//���ǵ�ĸ�����ȥ1
		bezierVolumeConfig.m = bezierVolumeConfig.ctrlPoints[0].size() - 1;
		bezierVolumeConfig.v = bezierVolumeConfig.ctrlPoints[0][0].size() - 1;
	}
}


Point3d BezierVolume::changeto3d(double x, double y)
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
void BezierVolume::mousePressEvent(QMouseEvent *event)
{
	Point3d p1 = changeto3d(event->pos().x(), event->pos().y());
	for (int i = 0; i <= bezierVolumeConfig.n; i++)
		for (int j = 0; j <= bezierVolumeConfig.m; j++)
		{
			Point3d p2 = bezierVolumeConfig.ctrlPoints[i][j];
			if (((p1.x >= p2.x - 4) && (p1.x <= p2.x + 4)) && ((p1.y >= p2.y - 4) && (p1.y <= p2.y + 4)) && ((p1.z >= p2.z - 4) && (p1.z <= p2.z + 4)))
			{
				flag = 1;
				ii = i;
				jj = j;
			}
		}
}

void BezierVolume::mouseMoveEvent(QMouseEvent *event){
	if (flag == 1){
		Point3d p = changeto3d(event->pos().x(), event->pos().y());
		bezierVolumeConfig.ctrlPoints[ii][jj] = p;
		updateGL();
	}
}
*/

void BezierVolume::mouseReleaseEvent(QMouseEvent *event) {
	flag = -1;
}


//������1���۲�����
void BezierVolume::mousePressEvent(QMouseEvent *event) {
	lastPos = event->pos();
	flag = 1;
}

void BezierVolume::mouseMoveEvent(QMouseEvent *event)
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
