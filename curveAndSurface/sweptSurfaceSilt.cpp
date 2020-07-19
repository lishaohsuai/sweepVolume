#include "SweptSurfaceSilt.h"
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
SweptSurfaceTransformSilt::SweptSurfaceTransformSilt(QWidget *parent) :
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
SweptSurfaceTransformSilt::~SweptSurfaceTransformSilt() {}

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
	Cw.x = x; Cw.y = y; Cw.z = z; double Cww = w; // Cww 新增加的一个维度
	// 计算 下面的 w
	if (Cww == 0) {
		std::cout << "Cww== 0 " << Cww << std::endl;
		exit(0);
	}
	C.x = Cw.x / Cww; C.y = Cw.y / Cww; C.z = Cw.z / Cww;
}



/**************************************************
@brief   : 简单的讲生成的点加入到曲线上
@author  : lee
@input   ：str  文件路径
@output  ：none
@time    : none
**************************************************/
void SweptSurfaceTransformSilt::genPoints(std::string str) {
	jsonReader(str);
	std::cout << "[DEBUG]genPoint" << std::endl;
	vector3d Bv(1, 0, 0);
	Matrix33 identity = Matrix33::getIdentityMatrix();
	sweptEntity.getSweepSurfaceSilt(T, C, S, 1000);// 得到两条曲线生成的一个曲面的结构体
	sweptEntity.outputSweptSur(cfg.P_Power, cfg.Q_Power, cfg.U_vector, cfg.V_vector, cfg.ctrlPoints);//输出这个结构体，然后绘制平面
	cfg.surfacePointsNumberRow = 50;
	cfg.surfacePointsNumberCol = 50;
	for (int i = 0; i < cfg.surfacePointsNumberRow; i++) {
		double u = i * 1.0 * (cfg.U_vector[cfg.U_vector.size() - 1] - cfg.U_vector[0]) / cfg.surfacePointsNumberRow;//均匀分成多少个点再所有的节点上
		std::vector<Point3d> temp;
		for (int j = 0; j < cfg.surfacePointsNumberCol; j++) {
			double v = j * 1.0 * (cfg.V_vector[cfg.V_vector.size() - 1] - cfg.V_vector[0]) / cfg.surfacePointsNumberCol;//均匀分成多少个点再所有的节点上
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
@brief   : 生成曲面上的点
@author  : lee
@input   : n+1 控制点的个数
		   p 阶次
		   U 节点向量
			m+1 节点的个数
		   m+1 另一个控制点的个数
		   q 另一个阶次
		   V 另一个节点向量
		   P  控制点数组 二维数组
		   u，v 从小到大，从而绘制出整个曲线
		   S 生成的点的坐标
@output  ：none
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
@brief   : 从文件中读取相应的参数
@author  : lee
@input   ：文件名(完整路径)/或空 方便调试
@output  ：none
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
		// 使用 结构体初始化两条曲线的结构体 (int p_, int n_, std::vector<double> U_, std::vector<Point3d> controlPts_) :
		BSplineCurveAbstract tmp(nurbsDoCvCfg.P_Power, nurbsDoCvCfg.ctrlPoints1.size() - 1, nurbsDoCvCfg.U_vector, nurbsDoCvCfg.ctrlPoints1);
		T.set(tmp);
	
		// move the section curve to origin // QU:为什么要把section Curve 移动到原点
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
@brief   : 初始化opengl环境
@author  : lee
@input   ：none
@output  ：none
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
@brief   : 改变窗口的大小
@author  : lee
@input   ：width 宽度
		   height 高度
@output  ：none
@time    : none
**************************************************/
void SweptSurfaceTransformSilt::resizeGL(int width, int height)
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
@input   : none
@output  : none
@time    : none
**************************************************/
void SweptSurfaceTransformSilt::paintGL() {
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
	//for (int i = 0; i < cfg.ctrlPoints.size(); i++) {
	//	for (int j = 0; j < cfg.ctrlPoints[0].size(); j++) {
	//		glVertex3d(cfg.ctrlPoints[i][j].x, cfg.ctrlPoints[i][j].y, cfg.ctrlPoints[i][j].z);
	//	}
	//}
	//glEnd();
	// 绘制坐标轴
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0, 0.0, 1.0); // 蓝色表示x
	glVertex3d(0, 0, 0);
	glVertex3d(1000, 0, 0);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0, 1.0, 0.0);// 绿色  表示 y
	glVertex3d(0, 0, 0);
	glVertex3d(0, 1000, 0);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glColor3f(1.0, 0, 0.0);// 红色表示的是 Z
	glVertex3d(0, 0, 0);
	glVertex3d(0, 0, 1000);
	glEnd();
	// 画出扫掠线
	glBegin(GL_LINE_STRIP);
	glColor3f(1.0, 1.0, 0.0);
	for (int i = 0; i < curvePoints.size(); i++) {
		glVertex3d(curvePoints[i].x, curvePoints[i].y, curvePoints[i].z);

	}
	glEnd();

	// 画出网格曲线
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
@brief   : 按键按下事件
@author  : lee
@input   ：e  事件
@output  ：none
@time    : none
**************************************************/
void SweptSurfaceTransformSilt::mousePressEvent(QMouseEvent *e) {
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
void SweptSurfaceTransformSilt::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : none
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/
void SweptSurfaceTransformSilt::mouseMoveEvent(QMouseEvent *e) {
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


