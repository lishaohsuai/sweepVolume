#include "bsplineVolume.h"
#include <algorithm>


//参考 The NURBS Book 2nd (38/660)
BSplineVolume::BSplineVolume(QWidget *parent)//就是确定控制顶点数目和控制顶点
	:Strategy(), QGLWidget(parent) {
	rotationX = 0;
	rotationY = 0;
	rotationZ = 0;
	flag = -1;
	times = 1.0;
}


/**************************************************
@brief   : 析构函数释放相关资源
@author  : lee
@input   ：none
@output  ：none
@time    : none
**************************************************/
BSplineVolume::~BSplineVolume() {}


/**************************************************
@brief   : 找到 u 所属于的区间序列
@author  : lee
@input   : n n = m - p -1
		   p 基函数的阶
		   u 递增的一般从0到1上的点
		   U 节点向量 U_0 - U_m
@output  : 节点区间序列
@time    : none
**************************************************/
//int BSplineVolume::findSpan(int n, int p, double u, const std::vector<double> &U) {
//	// 总觉得这个有问题 会陷入死循环
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
@brief   : 计算非空的基函数
		   第一次，p=0时，只有N[0]=1.0,即N0,0=1.0;p=1时，求N[0],N[1],即N0,1和N1,1;p=2时，求N[0],N[1],N[2],即N0,2  N1,2和N2,2
@author  : lee
@input   ：i 节点向量的第i个序列
		   u 生成曲线上的点
		   p 阶次
		   U 节点向量
@output  ：N 从第0阶到后面的阶辐射的三角形所对应的阶函数的值，一整套
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
@brief   : 生成体上的点  公式法
@author  : lee
@input   ：
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
		 P 控制点
		 x
		 y
		 z
@output  ：V
@time    : none
**************************************************/
//void BSplineVolume::volumePoint(int n0, int p0, const std::vector<double>&U0, int n1, int p1, const std::vector<double> &U1,
//	int n2, int p2, const std::vector<double> &U2, std::vector<std::vector<std::vector<Point3d>>>&P, 
//	double x, double y, double z, Point3d &V) {
//	// 似乎这个有问题
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
@brief   : 使用公式法生成BSpline Volume的点
@author  : lee
@input   ：none
@output  ：none
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

void BSplineVolume::resizeGL(int width, int height)
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
void BSplineVolume::paintGL() {

	//std::cout << "[DEBUG] BSplineVolume paintEvent" << std::endl;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// 清除屏幕和深度缓存
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//平移
	glRotatef(rotationX, 1.0, 0.0, 0.0);
	glRotatef(rotationY, 0.0, 1.0, 0.0);
	glRotatef(rotationZ, 0.0, 0.0, 1.0);//旋转
	glScalef(times, times, times);//缩放
	// 画出控制点
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
	// 画出表面
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
		bsplineVCfg.n0 = bsplineVCfg.ctrlPoints.size() - 1;//阶是点的个数减去1
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


//鼠标操作1，观察曲面
void BSplineVolume::mousePressEvent(QMouseEvent *event) {
	lastPos = event->pos();
	flag = 1;
}



/**************************************************
@brief   : 鼠标滚轮事件
@author  : lee
@input   ：event
@output  ：none
@time    : none
**************************************************/
void BSplineVolume::wheelEvent(QWheelEvent *event) {
	if (event->delta() > 0) {// 当滚轮远离使用者时
		times += 0.008f;
		update();
	}
	else {//当滚轮向使用者方向旋转时
		times -=0.008f;
		update();
	}
}

void BSplineVolume::mouseMoveEvent(QMouseEvent *event)
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
