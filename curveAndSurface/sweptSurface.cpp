#include "sweptSurface.h"
#include <QFileDialog>
#include <fstream>
#include <QGLWidget>
#include <gl/glu.h>

/**************************************************
@brief   : 扫掠面的实现步骤
		     先生成两条Nurbs曲线
			 曲线v沿着曲线u运动
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/

/**************************************************
@brief   : 构造函数 简单的初始化常量赋值
@author  : lee
@input   ：parent 一般为NULL
@output  ：none
@time    : none
**************************************************/
SweptSurface::SweptSurface(QWidget *parent) :
	Strategy(), QGLWidget(parent) {
	flag = -1;// 没有任何的鼠标事件
	rotationX = 0;
	rotationY = 0;
	rotationZ = 0;
};



/**************************************************
@brief   : 析构函数释放相关资源
@author  : lee
@input   ：none
@output  ：none
@time    : none
**************************************************/
SweptSurface::~SweptSurface() {}

/**************************************************
@brief   : 找到 u 所属于的区间序列
@author  : lee
@input   : numOfCtlPoint 控制点的个数
		   order 基函数的阶
		   u 递增的一般从0到1上的点
		   U 节点向量
@output  : 节点区间序列
@time    : none
**************************************************/
int SweptSurface::findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U) {
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
@brief   : 计算非空的基函数
		   第一次，p=0时，只有N[0]=1.0,即N0,0=1.0;p=1时，求N[0],N[1],即N0,1和N1,1;p=2时，求N[0],N[1],N[2],即N0,2  N1,2和N2,2
@author  : lee
@input   ：i 节点向量的第i个序列
		   u 生成曲线上的点
		   order 阶次
		   U 节点向量
@output  ：base 从第0阶到后面的阶辐射的三角形所对应的阶函数的值，一整套
@time    : none
**************************************************/
void SweptSurface::basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base) {
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
@brief   : 生成曲线上的点
		   关键公式  m = n + p + 1
@author  : lee
@input   : n+1 控制点的个数
		   p 阶次
		   m+1 节点的个数
		   P  控制点数组
		   u 从小到大，从而绘制出整个曲线
		   C 生成的点的坐标
@output  ：none
@time    : none
**************************************************/
void SweptSurface::curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u,
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
	Cw.x = x; Cw.y = y; Cw.z = z; double Cww = w; // Cww 新增加的一个维度
	// 计算 下面的 w
	if (Cww == 0) {
		std::cout << "Cww== 0 " << Cww << std::endl;
		exit(0);
	}
	C.x = Cw.x / Cww; C.y = Cw.y / Cww; C.z = Cw.z / Cww;
}



/**************************************************
@brief   : 扫掠生成扫掠面
@author  : lee
@input   : n 控制点的个数
		   p 阶次
		   U 节点向量
		   m+1 另一个线控制点的个数
		   q 阶次
		   V 节点向量
		   P 控制点数组
		   Q 控制点数组
		   u 从小到大，从而绘制出整个曲线
		   C 生成的点的坐标
@output  ：none
@time    : none
**************************************************/
void SweptSurface::surfacePoint(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V,
	const std::vector<Point3d> &P, const std::vector<Point3d> &Q, double u, double v, Point3d &S,
	const std::vector<double> &weight1, const std::vector<double> &weight2) {
	int uspan = findSpan(n, p, u, U);
	std::vector<double> baseu;
	basisFuns(uspan, u, p, U, baseu);
	std::vector<double> basev;
	int vspan = findSpan(m, q, v, V);
	basisFuns(vspan, v, q, V, basev);
	std::vector<Point4d> temp;
	for (int l = 0; l <= q; l++) {
		Point4d tmp;
		for (int k = 0; k <= p; k++) {
			tmp.x = tmp.x + baseu[k] * (P[uspan - p + k].x + Q[vspan - q + l].x) * weight1[uspan - p + k] * weight2[vspan - q + l];
			tmp.y = tmp.y + baseu[k] * (P[uspan - p + k].y + Q[vspan - q + l].y) * weight1[uspan - p + k] * weight2[vspan - q + l];
			tmp.z = tmp.z + baseu[k] * (P[uspan - p + k].z + Q[vspan - q + l].z) * weight1[uspan - p + k] * weight2[vspan - q + l];
			tmp.w = tmp.w + baseu[k] * weight1[uspan - p + k] * weight2[vspan - q + l];
		}
		temp.push_back(tmp);
	}
	Point4d Sw;
	for (int l = 0; l <= q; l++) {
		Sw.x = Sw.x + basev[l] * temp[l].x;
		Sw.y = Sw.y + basev[l] * temp[l].y;
		Sw.z = Sw.z + basev[l] * temp[l].z;
		Sw.w = Sw.w + basev[l] * temp[l].w;
	}
	if (Sw.w == 0) {
		std::cout << "ERROR Sw.w == 0" << std::endl;
	}
	S.x = Sw.x / Sw.w;
	S.y = Sw.y / Sw.w;
	S.z = Sw.z / Sw.w;
}

/**************************************************
@brief   : 简单的讲生成的点加入到曲线上
@author  : lee
@input   ：str  文件路径
@output  ：none
@time    : none
**************************************************/
void SweptSurface::genPoints(std::string str) {
	jsonReader(str);

	std::cout << "[DEBUG]genPoint" << std::endl;
	/* 生成两个曲线实现但是没有必要
	nurbsDoCvCfg.curvaturePoints1.clear();
	for (int i = 0; i <= nurbsDoCvCfg.curvaturePointsNumber1; i++) {
		double u = i * 1.0 * (nurbsDoCvCfg.U_vector[nurbsDoCvCfg.U_vector.size() - 1] - nurbsDoCvCfg.U_vector[0]) / nurbsDoCvCfg.curvaturePointsNumber1;//均匀分成多少个点再所有的节点上
		Point3d C;
		curvePoint(nurbsDoCvCfg.ctrlPoints1.size() - 1, nurbsDoCvCfg.P_Power, nurbsDoCvCfg.U_vector, nurbsDoCvCfg.ctrlPoints1, u, C, nurbsDoCvCfg.weight1);
		nurbsDoCvCfg.curvaturePoints1.push_back(C);
	}
	nurbsDoCvCfg.curvaturePoints2.clear();
	for (int i = 0; i <= nurbsDoCvCfg.curvaturePointsNumber2; i++) {
		double u = i * 1.0 * (nurbsDoCvCfg.V_vector[nurbsDoCvCfg.V_vector.size() - 1] - nurbsDoCvCfg.V_vector[0]) / nurbsDoCvCfg.curvaturePointsNumber2;//均匀分成多少个点再所有的节点上
		Point3d C;
		curvePoint(nurbsDoCvCfg.ctrlPoints2.size() - 1, nurbsDoCvCfg.Q_Power, nurbsDoCvCfg.V_vector, nurbsDoCvCfg.ctrlPoints2, u, C, nurbsDoCvCfg.weight2);
		nurbsDoCvCfg.curvaturePoints2.push_back(C);
	}
	*/
	for (int i = 0; i < nurbsDoCvCfg.surfacePoints.size(); i++) {
		nurbsDoCvCfg.surfacePoints[i].clear();
	}
	nurbsDoCvCfg.surfacePoints.clear();
	for (int i = 0; i < nurbsDoCvCfg.curvaturePointsNumber1; i++) {
		double u = i * 1.0 * (nurbsDoCvCfg.U_vector[nurbsDoCvCfg.U_vector.size() - 1] - nurbsDoCvCfg.U_vector[0]) / nurbsDoCvCfg.curvaturePointsNumber1;
		std::vector<Point3d> temp;
		for (int j = 0; j < nurbsDoCvCfg.curvaturePointsNumber2; j++) {
			double v = j * 1.0 * (nurbsDoCvCfg.V_vector[nurbsDoCvCfg.V_vector.size() - 1] - nurbsDoCvCfg.V_vector[0]) / nurbsDoCvCfg.curvaturePointsNumber2;
			Point3d S;
			/*(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V,
					const std::vector<Point3d> &P, const std::vector<Point3d> &Q, double u, double v, Point3d &S,
					const std::vector<double> &weight1, const std::vector<double> &weight2)*/
			surfacePoint(nurbsDoCvCfg.ctrlPoints1.size(), nurbsDoCvCfg.P_Power, nurbsDoCvCfg.U_vector, nurbsDoCvCfg.ctrlPoints1.size(),
				nurbsDoCvCfg.Q_Power, nurbsDoCvCfg.V_vector, nurbsDoCvCfg.ctrlPoints1, nurbsDoCvCfg.ctrlPoints2, u, v, S, nurbsDoCvCfg.weight1, nurbsDoCvCfg.weight2);
			temp.push_back(S);
		}
		nurbsDoCvCfg.surfacePoints.push_back(temp);
	}
}



/**************************************************
@brief   : 从文件中读取相应的参数
@author  : lee
@input   ：文件名(完整路径)/或空 方便调试
@output  ：none
@time    : none
**************************************************/
void SweptSurface::jsonReader(std::string fileName) {
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
		nurbsDoCvCfg.curvaturePointsNumber1 = root["curvePointNum1"].asInt();
		nurbsDoCvCfg.curvaturePointsNumber2 = root["curvePointNum2"].asInt();
		nurbsDoCvCfg.surfacePointsNum = root["surfacePointsNum"].asInt();
		Json::Value points1 = root["ctrlPoint1"];
		for (int i = 0; i < points1.size(); i++) {
			Point3d p(0, 0, 0);
			p.x = (points1[i][0].asDouble());
			p.y = (points1[i][1].asDouble());
			p.z = (points1[i][2].asDouble());
			nurbsDoCvCfg.ctrlPoints1.push_back(p);
		}
		Json::Value points2 = root["ctrlPoint2"];
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
		Json::Value weight1 = root["weight1"];
		for (int i = 0; i < weight1.size(); i++) {
			nurbsDoCvCfg.weight1.push_back(weight1[i].asDouble());
		}
		Json::Value weight2 = root["weight2"];
		for (int i = 0; i < weight2.size(); i++) {
			nurbsDoCvCfg.weight2.push_back(weight2[i].asDouble());
		}
	}
}


/**************************************************
@brief   : 初始化opengl环境
@author  : lee
@input   ：none
@output  ：none
@time    : none
**************************************************/
void SweptSurface::initializeGL() {
	glShadeModel(GL_SMOOTH);
	glClearColor(0.1, 0.1, 0.4, 1.0);
	glClearDepth(1.0);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}


/**************************************************
@brief   : 改变窗口的大小
@author  : lee
@input   ：width 宽度
		   height 高度
@output  ：none
@time    : none
**************************************************/
void SweptSurface::resizeGL(int width, int height)
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


/**************************************************
@brief   : 自动和被动 调用更新整个图形
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/
void SweptSurface::paintGL() {
	//genPoints("normal");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// 清除屏幕和深度缓存
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//平移

	glRotated(rotationX, 1.0, 0.0, 0.0);
	glRotated(rotationY, 0.0, 1.0, 0.0);
	glRotated(rotationZ, 0.0, 0.0, 1.0);//旋转

	// 画控制点
	//glPointSize(8.0);
	//glBegin(GL_POINTS);
	//glColor3f(1.0, 0.0, 0.0);
	//for (int i = 0; i < nurbsDoCvCfg.ctrlPoints1.size(); i++) {
	//	glVertex3d(nurbsDoCvCfg.ctrlPoints1[i].x, nurbsDoCvCfg.ctrlPoints1[i].y, nurbsDoCvCfg.ctrlPoints1[i].z);
	//}
	//for (int i = 0; i < nurbsDoCvCfg.ctrlPoints2.size(); i++) {
	//	glVertex3d(nurbsDoCvCfg.ctrlPoints2[i].x, nurbsDoCvCfg.ctrlPoints2[i].y, nurbsDoCvCfg.ctrlPoints2[i].z);
	//}
	//glEnd();

	// 画出控制多边形
	//glBegin(GL_LINE_STRIP);
	//glColor3f(0.0, 1.0, 0.0);
	//for (int i = 0; i < nurbsDoCvCfg.ctrlPoints1.size(); i++) {
	//	glVertex3d(nurbsDoCvCfg.ctrlPoints1[i].x, nurbsDoCvCfg.ctrlPoints1[i].y, nurbsDoCvCfg.ctrlPoints1[i].z);
	//}
	//glEnd();
	//glBegin(GL_LINE_STRIP);
	//glColor3f(0.0, 1.0, 0.0);
	//for (int i = 0; i < nurbsDoCvCfg.ctrlPoints2.size(); i++) {
	//	glVertex3d(nurbsDoCvCfg.ctrlPoints2[i].x, nurbsDoCvCfg.ctrlPoints2[i].y, nurbsDoCvCfg.ctrlPoints2[i].z);
	//}
	//glEnd();

	// 画出曲线
	//glBegin(GL_LINE_STRIP);
	//glColor3f(0.0, 0.0, 1.0);
	//for (int i = 0; i < nurbsDoCvCfg.curvaturePoints1.size(); i++) {
	//	glVertex3f(nurbsDoCvCfg.curvaturePoints1[i].x, nurbsDoCvCfg.curvaturePoints1[i].y, nurbsDoCvCfg.curvaturePoints1[i].z);
	//}
	//glEnd();
	//glBegin(GL_LINE_STRIP);
	//glColor3f(0.0, 0.0, 1.0);
	//for (int i = 0; i < nurbsDoCvCfg.curvaturePoints2.size(); i++) {
	//	glVertex3f(nurbsDoCvCfg.curvaturePoints2[i].x, nurbsDoCvCfg.curvaturePoints2[i].y, nurbsDoCvCfg.curvaturePoints2[i].z);
	//}
	//glEnd();
	for (int i = 0; i < nurbsDoCvCfg.surfacePoints.size(); i++) {
		glBegin(GL_LINE_STRIP);
		glColor3f(1.0, 1.0, 1.0);
		for (int j = 0; j < nurbsDoCvCfg.surfacePoints[0].size(); j++) {
			glVertex3d(nurbsDoCvCfg.surfacePoints[i][j].x, nurbsDoCvCfg.surfacePoints[i][j].y, nurbsDoCvCfg.surfacePoints[i][j].z);
		}
		glEnd();
	}
	for (int i = 0; i < nurbsDoCvCfg.surfacePoints[0].size(); i++) {
		glBegin(GL_LINE_STRIP);
		glColor3f(1.0, 1.0, 1.0);
		for (int j = 0; j < nurbsDoCvCfg.surfacePoints.size(); j++) {
			glVertex3d(nurbsDoCvCfg.surfacePoints[j][i].x, nurbsDoCvCfg.surfacePoints[j][i].y, nurbsDoCvCfg.surfacePoints[j][i].z);
		}
		glEnd();
	}
}



/**************************************************
@brief   : 按键按下事件
@author  : lee
@input   ：e  事件
@output  ：none
@time    : none
**************************************************/
void SweptSurface::mousePressEvent(QMouseEvent *e) {
	lastPos = e->pos();
	flag = 1;
}


/**************************************************
@brief   : none
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/
void SweptSurface::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : none
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/
void SweptSurface::mouseMoveEvent(QMouseEvent *e) {
	if (flag) {
		GLdouble dx = GLdouble(e->x() - lastPos.x()) / width();//QWidght 的 宽度
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


