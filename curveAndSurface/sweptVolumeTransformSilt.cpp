#include "SweptVolumeTransformSilt.h"
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
SweptVolumeTransformSilt::SweptVolumeTransformSilt(QWidget *parent) :
	Strategy(), QGLWidget(parent) {
	flag = -1;// 没有任何的鼠标事件
	rotationX = 0;
	rotationY = 0;
	rotationZ = 0;
	times = 1.0;
	TcurveCCPoints = 1000;
	Planar = false;
};



/**************************************************
@brief   : 析构函数释放相关资源
@author  : lee
@input   ：none
@output  ：none
@time    : none
**************************************************/
SweptVolumeTransformSilt::~SweptVolumeTransformSilt() {}

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
int SweptVolumeTransformSilt::findSpan(int n, int p, double u, const std::vector<double> &U) {
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
void SweptVolumeTransformSilt::basisFuns(int i, double u, int p, const std::vector<double> &U, std::vector<double> &N) {
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
@brief   : 简单的讲生成的点加入到曲线上
@author  : lee
@input   ：str  文件路径
@output  ：none
@time    : none
**************************************************/
void SweptVolumeTransformSilt::genPoints(std::string str) {
	jsonReader(str);
	std::cout << "[DEBUG]genPoint" << std::endl;
	sweptEntity.getSweepVolumeSilt(T, Suv, Sx, Sy, Twist, TcurveCCPoints);// 得到两条曲线生成的一个曲面的结构体
	sweptEntity.outputSweptVolume(cfg.p0, cfg.p1, cfg.p2, cfg.U0, cfg.U1, cfg.U2, cfg.ctrlPoints);//输出这个结构体，然后绘制平面
	cfg.genPointNumX = 20;
	cfg.genPointNumY = 20;
	cfg.genPointNumZ = 100;
	cfg.n2 = cfg.ctrlPoints.size() - 1;
	cfg.n1 = cfg.ctrlPoints[0].size() - 1;
	cfg.n0 = cfg.ctrlPoints[0][0].size() - 1;
	int count = 0;
	for (int i = 0; i < cfg.genPointNumX; i++) {
		std::vector<std::vector<Point3d>> surfacePoints;
		for (int j = 0; j < cfg.genPointNumY; j++) {
			std::vector<Point3d> linePoints;
			for (int k = 0; k < cfg.genPointNumZ; k++) {
				double u = i * 1.0 * (cfg.U0[cfg.U0.size() - 1] - cfg.U0[0]) / cfg.genPointNumX;//均匀分成多少个点再所有的节点上
				double v = j * 1.0 * (cfg.U1[cfg.U1.size() - 1] - cfg.U1[0]) / cfg.genPointNumY;//均匀分成多少个点再所有的节点上
				double w = k * 1.0 * (cfg.U2[cfg.U2.size() - 1] - cfg.U2[0]) / cfg.genPointNumZ;//均匀分成多少个点再所有的节点上
				Point3d V;
				//std::cout << "[DEBUG] 开始输出 " << count++ <<std::endl;
				volumePoint(cfg.n0, cfg.p0, cfg.U0,
					cfg.n1, cfg.p1, cfg.U1,
					cfg.n2, cfg.p2, cfg.U2,
					cfg.ctrlPoints, u, v, w, V);// n2 是长度
				//std::cout << "[DEBUG] 中间崩溃" << std::endl;
				linePoints.push_back(V);
			}
			surfacePoints.push_back(linePoints);
		}
		cfg.volumePoints.push_back(surfacePoints);
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
void SweptVolumeTransformSilt::curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u, Point3d &C) {
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
@time    : none
**************************************************/
void SweptVolumeTransformSilt::volumePoint(int n0, int p0, const std::vector<double>&U0, int n1, int p1, const std::vector<double> &U1,
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
@brief   : 从文件中读取相应的参数
@author  : lee
@input   ：文件名(完整路径)/或空 方便调试
@output  ：none
@time    : none
**************************************************/
void SweptVolumeTransformSilt::jsonReader(std::string fileName) {
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
		// ---- 轨迹曲线 ----
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
		TcurveCCPoints = root["curvaturePointsNumber"].asInt();

		// 使用 结构体初始化两条曲线的结构体 (int p_, int n_, std::vector<double> U_, std::vector<Point3d> controlPts_) :
		BSplineCurveAbstract bcu(CP, ccPoint.size() - 1, CU, ccPoint);
		T.set(bcu);
		// move the section curve to origin // QU:为什么要把section Curve 移动到原点
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
			double u = i * 1.0 * (CU[CU.size() - 1] - CU[0]) / curvaturePointsNumber;//均匀分成多少个点再所有的节点上
			Point3d C;
			curvePoint(ccPoint.size() - 1, CP, CU, ccPoint, u, C);
			curvePoints.push_back(C);
		}
		// ---- scale曲线 改为 缩放函数 Sz  并不需要----
		Sx.set(jsonReaderScale("scaleCtrlPointsx", "ScaleUx_vector", "ScalePx_Power", root));
		Sy.set(jsonReaderScale("scaleCtrlPointsy", "ScaleUy_vector", "ScalePy_Power", root));
		// ---- twist曲线 twist  并不需要----
		Twist.set(jsonReaderScale("twistCtrlPoints", "TwistU_vector", "TwistP_Power", root));
		// ---- Planar ----
		Planar = root["Planar"].asBool();
	}

}



/**************************************************
@brief   : 读取缩放曲线函数
@author  : lee
@input   ：参数可以看出  不写
@output  ：none
@time    : none
**************************************************/
BSplineCurveAbstract SweptVolumeTransformSilt::jsonReaderScale(std::string ctrlPointsStr, std::string UVectorStr, std::string PStr, Json::Value &root) {
	Json::Value vectorsScalex = root[ctrlPointsStr];
	std::vector<Point3d> scalecPointx;
	for (int i = 0; i < vectorsScalex.size(); i++) {
		Point3d point;
		point.x = vectorsScalex[i][0].asDouble();
		point.y = vectorsScalex[i][1].asDouble();
		point.z = 0;
		scalecPointx.push_back(point);
	}
	Json::Value ScaleCurveUJx = root[UVectorStr];
	std::vector<double> scalecUx;
	for (int i = 0; i < ScaleCurveUJx.size(); i++) {
		scalecUx.push_back(ScaleCurveUJx[i].asDouble());
	}
	int scaleCurvePowerx = root[PStr].asInt();
	BSplineCurveAbstract temp(scaleCurvePowerx, scalecPointx.size() - 1, scalecUx, scalecPointx);
	return temp;
}

/**************************************************
@brief   : 初始化opengl环境
@author  : lee
@input   ：none
@output  ：none
@time    : none
**************************************************/
void SweptVolumeTransformSilt::initializeGL() {
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
void SweptVolumeTransformSilt::resizeGL(int width, int height)
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
void SweptVolumeTransformSilt::paintGL() {
	//genPoints("normal");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// 清除屏幕和深度缓存
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//平移

	glRotated(rotationX, 1.0, 0.0, 0.0);
	glRotated(rotationY, 0.0, 1.0, 0.0);
	glRotated(rotationZ, 0.0, 0.0, 1.0);//旋转
	glScalef(times, times, times);//缩放
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
	// 画出控制点
	/*glPointSize(8.f);
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
	glEnd();*/
	// 画出表面
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
	// 画出垂直的线条
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
	//画出 轨迹曲线
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
@brief   : 按键按下事件
@author  : lee
@input   ：e  事件
@output  ：none
@time    : none
**************************************************/
void SweptVolumeTransformSilt::mousePressEvent(QMouseEvent *e) {
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
void SweptVolumeTransformSilt::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : 鼠标滚轮事件
@author  : lee
@input   ：event
@output  ：none
@time    : none
**************************************************/
void SweptVolumeTransformSilt::wheelEvent(QWheelEvent *event) {
	if (event->delta() > 0) {// 当滚轮远离使用者时
		times += 0.008f;
		update();
	}
	else {//当滚轮向使用者方向旋转时
		times -= 0.008f;
		update();
	}
}



/**************************************************
@brief   : none
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/
void SweptVolumeTransformSilt::mouseMoveEvent(QMouseEvent *e) {
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


/**************************************************
@brief   : 导出obj文件
@author  : lee
@input   ：none
@output  ：none
@time    : none
**************************************************/
void SweptVolumeTransformSilt::exportOBJ() {
	MyMesh mesh;
	int z = cfg.volumePoints.size();
	int x = cfg.volumePoints[0].size();
	int y = cfg.volumePoints[0][0].size();
	int faceDir[][2] = { {0, z - 1},{0, x - 1},{0, y - 1} };

	for (int t = 0; t < 1; t++) {
		std::vector<std::vector<Point3d>> v;//底面
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
		std::vector<std::vector<Point3d>> v;//底面
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
		std::vector<std::vector<Point3d>> v;//侧面1
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
		std::vector<std::vector<Point3d>> v;//侧面1
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
		std::vector<std::vector<Point3d>> v;//侧面2
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
		std::vector<std::vector<Point3d>> v;//侧面2
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
	std::cout << "输出完毕" << std::endl;
}



/**************************************************
@brief   : 生成一个面的mesh
@author  : lee
@input   ：mesh 网格主体
		   m 行
		   n 列
		   v 二维数组点坐标
@output  ：none
@time    : none
**************************************************/
void SweptVolumeTransformSilt::genOneFaceMesh(MyMesh &mesh, int m, int n, const std::vector<std::vector<Point3d>> &v) {
	MyMesh::VertexHandle **vhandle = new MyMesh::VertexHandle*[m];
	for (int i = 0; i < m; i++) {
		vhandle[i] = new MyMesh::VertexHandle[n];//二维数组句柄
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
			face_vhandles.push_back(vhandle[i + 1][j]);//加入两个面
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
@brief   : 解决面的相反性
@author  : lee
@input   : mesh 网格主体
		   m 行
		   n 列
		   v 二维数组点坐标
@output  ：none
@time    : none
**************************************************/
void SweptVolumeTransformSilt::genOtherFaceMesh(MyMesh &mesh, int m, int n, const std::vector<std::vector<Point3d>> &v) {
	MyMesh::VertexHandle **vhandle = new MyMesh::VertexHandle*[m];
	for (int i = 0; i < m; i++) {
		vhandle[i] = new MyMesh::VertexHandle[n];//二维数组句柄
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
			face_vhandles.push_back(vhandle[i + 1][j]);//加入两个面
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


