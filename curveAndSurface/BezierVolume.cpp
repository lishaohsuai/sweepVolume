#include "BezierVolume.h"
#include <algorithm>


//参考 The NURBS Book 2nd (38/660)
BezierVolume::BezierVolume(QWidget *parent)//就是确定控制顶点数目和控制顶点
	:Strategy(), QGLWidget(parent) {
	rotationX = 0;
	rotationY = 0;
	rotationZ = 0;
	flag = -1;
}

BezierVolume::~BezierVolume() {}


//生成平面上对应的点
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
@brief   : 计算阶层
@author  : lee
@input   : u 从0到1的数
		   n 表示几阶
		   i 表示现在算的是第几阶
		   factorialN 表示 阶乘 公式的计算
@output  ：none
@time    : none
**************************************************/
double BezierVolume::calcBernstein(double u, int n, int i, std::vector<int> &factorialN) {
	return factorialN[n] * pow(u, i)*pow((1 - u), n - i) / (double)(factorialN[i] * factorialN[n - i]);
}



/**************************************************
@brief   : 计算阶乘
@author  : lee
@input   ：factorialN 空数组
		   n  总共的阶乘数
@output  ：none
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
@brief   : 使用公式法生成Bezier Volume的点
@author  : lee
@input   ：none
@output  ：none
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
	// 启用smooth shading(阴影平滑)
	glShadeModel(GL_SMOOTH);
	// 设置清楚屏幕所用的颜色 色彩值范1围从0.0-1.0 (R G B A) 清屏时A不起作用
	glClearColor(0.1, 0.1, 0.4, 1.0);
	/* 以下三行必须做,是关于深度缓存的,深度缓存是OpenGL十分重要的部分*/
	// 设置深度缓存
	glClearDepth(1.0);
	// 启用深度测试
	glEnable(GL_DEPTH_TEST);
	// 设置深度测试的类型
	glDepthFunc(GL_LEQUAL);
	// 设置希望得到最好的透视修正
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

void BezierVolume::resizeGL(int width, int height)
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
// (X坐标轴从左至右，Y坐标轴从下至上，Z坐标轴从里至外)
// OpenGL屏幕中心的坐标值是X和Y轴上的0.0点
// 中心左面的坐标值是负值，右面是正值。
// 移向屏幕顶端是正值，移向屏幕底端是负值。
// 移入屏幕深处是负值，移出屏幕则是正值。
void BezierVolume::paintGL() {

	//std::cout << "[DEBUG] BezierVolume paintEvent" << std::endl;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// 清除屏幕和深度缓存
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//平移
	glRotatef(rotationX, 1.0, 0.0, 0.0);
	glRotatef(rotationY, 0.0, 1.0, 0.0);
	glRotatef(rotationZ, 0.0, 0.0, 1.0);//旋转
	// 画出控制点
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
	// 画出表面
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
		bezierVolumeConfig.n = bezierVolumeConfig.ctrlPoints.size() - 1;//阶是点的个数减去1
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
	glGetIntegerv(GL_VIEWPORT, viewport); // 获取视图矩阵
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);//获取模型视图矩阵
	glGetDoublev(GL_PROJECTION_MATRIX, projection);//获取投影矩阵
	glPopMatrix();//出栈，恢复刚刚入栈的矩阵信息
	winX = x;
	winY = width() - y;    //width为屏幕的宽度（在Qt中需要更改为height，既高度）   
	//好像是通过读取像素的方式获得z坐标
	glReadPixels((int)winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);//读取屏幕深度  
	//坐标转换
	gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
	Point3d pos = { posX,posY,posZ };
	return pos;
}

/*
//鼠标操作2，拖动控制顶点
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


//鼠标操作1，观察曲面
void BezierVolume::mousePressEvent(QMouseEvent *event) {
	lastPos = event->pos();
	flag = 1;
}

void BezierVolume::mouseMoveEvent(QMouseEvent *event)
{
	if (flag == 1)
	{
		//为什么这么计算，不太了解，先放着
		GLfloat dx = GLfloat(event->x() - lastPos.x()) / width();
		GLfloat dy = GLfloat(event->y() - lastPos.y()) / height();
		if (event->buttons() & Qt::LeftButton) {
			rotationX += 180 * dy;
			rotationY += 180 * dx;
			updateGL();//每次都要重新计算点，反应有些慢
		}
		else if (event->buttons() & Qt::RightButton) {
			rotationX += 180 * dy;
			rotationZ += 180 * dx;
			updateGL();
		}
		lastPos = event->pos();
	}
}
