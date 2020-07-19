
#include "bezierSurface.h"

//�ο� The NURBS Book 2nd (38/660)
BezierSurface::BezierSurface(QWidget *parent)//����ȷ�����ƶ�����Ŀ�Ϳ��ƶ���
	:Strategy(),QGLWidget(parent) {
	rotationX = 0;
	rotationY = 0;
	rotationZ = 0;
	flag = -1;
}

BezierSurface::~BezierSurface() {}


//����ƽ���϶�Ӧ�ĵ�
void BezierSurface::calcDeCasteljau2(const std::vector<std::vector<Point3d> > &P, int n, int m, double u0
	, double v0, Point3d &S) {
	std::vector<Point3d> Q;
	Q.resize(m+1);
	for (int j = 0; j <= m; j++) {
		calcDeCasteljau1(P[j], n, u0, Q[j]);
	}
	calcDeCasteljau1(Q, m, v0, S);
}

void BezierSurface::calcDeCasteljau1(const std::vector<Point3d> &P, int n, double u, Point3d & _Q){
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


void BezierSurface::genPoints(std::string method) {
	jsonReader();
	bezierSurfaceConfig.surfacePoints.clear();
	for (int index = 0; index <= bezierSurfaceConfig.genPointNumX; index++) {
		double u = (double)index / (double)bezierSurfaceConfig.genPointNumX;
		std::vector<Point3d> vPoints;
		Point3d p(0, 0, 0);
		for (int k = 0; k <= bezierSurfaceConfig.genPointNumY; k++) {
			double v = (double)k / (double)bezierSurfaceConfig.genPointNumY;
			calcDeCasteljau2(bezierSurfaceConfig.ctrlPoints, bezierSurfaceConfig.n, bezierSurfaceConfig.m, u, v, p);
			vPoints.push_back(p);
		}
		bezierSurfaceConfig.surfacePoints.push_back(vPoints);
	}
}

void BezierSurface::initializeGL()
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

void BezierSurface::resizeGL(int width, int height)
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
void BezierSurface::paintGL() {

	//std::cout << "[DEBUG] BezierSurface paintEvent" << std::endl;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// �����Ļ����Ȼ���
	//genPoints("NULL");
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//ƽ��
	glRotatef(rotationX, 1.0, 0.0, 0.0);
	glRotatef(rotationY, 0.0, 1.0, 0.0);
	glRotatef(rotationZ, 0.0, 0.0, 1.0);//��ת
	// �������Ƶ�
	glPointSize(8.f);
	for (int i = 0; i < bezierSurfaceConfig.ctrlPoints.size(); i++)
	{
		glBegin(GL_POINTS);
		glColor3f(0.0, 1.0, 0.0);
		for (int j = 0; j < bezierSurfaceConfig.ctrlPoints[i].size(); j++)
			glVertex3f(bezierSurfaceConfig.ctrlPoints[i][j].x, bezierSurfaceConfig.ctrlPoints[i][j].y,
				bezierSurfaceConfig.ctrlPoints[i][j].z);
		glEnd();
	}
	// ��������
	for (int j = 0; j < bezierSurfaceConfig.genPointNumX; j++)
	{
		glBegin(GL_LINE_STRIP);
		glColor3f(0.0, 0.0, 1.0);
		for (int i = 0; i < bezierSurfaceConfig.genPointNumY; i++)
			glVertex3f(bezierSurfaceConfig.surfacePoints[i][j].x, bezierSurfaceConfig.surfacePoints[i][j].y, 
				bezierSurfaceConfig.surfacePoints[i][j].z);
		glEnd();
	}
	for (int i = 0; i < bezierSurfaceConfig.genPointNumX; i++)
	{
		glBegin(GL_LINE_STRIP);
		glColor3f(0.0, 0.0, 1.0);
		for (int j = 0; j < bezierSurfaceConfig.genPointNumY; j++)
			glVertex3f(bezierSurfaceConfig.surfacePoints[i][j].x, bezierSurfaceConfig.surfacePoints[i][j].y, 
				bezierSurfaceConfig.surfacePoints[i][j].z);
		glEnd();
	}
}

void BezierSurface::jsonReader() {
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
		bezierSurfaceConfig.genPointNumX = root["genPointNumX"].asInt();
		bezierSurfaceConfig.genPointNumY = root["genPointNumY"].asInt();
		bezierSurfaceConfig.name = root["name"].asString();
		Json::Value points = root["Point"];
		struct Point3d p(0, 0, 0);
		for (int i = 0; i < points.size(); i++) {
			Json::Value pointsRow = points[i];
			std::vector<struct Point3d> v;
			for (int j = 0; j < pointsRow.size(); j++) {
				p.x = pointsRow[j][0].asDouble();
				p.y = pointsRow[j][1].asDouble();
				p.z = pointsRow[j][2].asDouble();
				v.push_back(p);
			}
			bezierSurfaceConfig.ctrlPoints.push_back(v);
		}
		bezierSurfaceConfig.n = bezierSurfaceConfig.ctrlPoints.size() - 1;//���ǵ�ĸ�����ȥ1
		bezierSurfaceConfig.m = bezierSurfaceConfig.ctrlPoints[0].size() - 1;
	}
}


Point3d BezierSurface::changeto3d(double x, double y)
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
void BezierSurface::mousePressEvent(QMouseEvent *event)
{
	Point3d p1 = changeto3d(event->pos().x(), event->pos().y());
	for (int i = 0; i <= bezierSurfaceConfig.n; i++)
		for (int j = 0; j <= bezierSurfaceConfig.m; j++)
		{
			Point3d p2 = bezierSurfaceConfig.ctrlPoints[i][j];
			if (((p1.x >= p2.x - 4) && (p1.x <= p2.x + 4)) && ((p1.y >= p2.y - 4) && (p1.y <= p2.y + 4)) && ((p1.z >= p2.z - 4) && (p1.z <= p2.z + 4)))
			{
				flag = 1;
				ii = i;
				jj = j;
			}
		}
}

void BezierSurface::mouseMoveEvent(QMouseEvent *event){
	if (flag == 1){
		Point3d p = changeto3d(event->pos().x(), event->pos().y());
		bezierSurfaceConfig.ctrlPoints[ii][jj] = p;
		updateGL();
	}
}
*/

void BezierSurface::mouseReleaseEvent(QMouseEvent *event){
	flag = -1;
}


//������1���۲�����
void BezierSurface::mousePressEvent(QMouseEvent *event){
	lastPos = event->pos();
	flag = 1;
}

void BezierSurface::mouseMoveEvent(QMouseEvent *event)
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
