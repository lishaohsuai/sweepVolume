#include "sweptVolume.h"
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
SweptVolume::SweptVolume(QWidget *parent) :
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
SweptVolume::~SweptVolume() {}

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
int SweptVolume::findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U) {
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
void SweptVolume::basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base) {
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
void SweptVolume::curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u,
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
@input   : n+1 控制点的个数
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
void SweptVolume::surfacePoint(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V,
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
@brief   : 生成扫掠体
@author  : lee
@input   : cn+1 轨迹线控制点的个数
		   cp 轨迹线的阶次
		   cU 轨迹线的节点向量
		   curveWeight 曲线的权重
		   cP 轨迹线的控制点
		   k 从小到大变化
		   --
		   sn+1 行控制点的个数
		   sp 行控制点的阶次
		   sU 行控制点的节点向量
		   sm+1 列控制点的个数
		   sq 列控制点的阶次
		   sV 列控制点的节点向量
		   sP 控制顶点
		   u,v 从小到大变化
		   surfaceWeight 面控制点的权重
		   --
		   S 生成的顶点
@output  ：none
@time    : none
**************************************************/
void SweptVolume::volumePoint(int cn, int cp, const std::vector<double>&cU, const std::vector<double> &curveWeight, const std::vector<Point3d> &cP, double k,
	int sn, int sp, const std::vector<double> &sU,
	int sm, int sq, const std::vector<double> &sV,
	std::vector<std::vector<Point3d> >&sP, double u, double v,const std::vector<std::vector<double> > surfaceWeight, Point3d &S ) {
	int cuspan = findSpan(cn, cp, k, cU);
	std::vector<double> basecv;
	basisFuns(cuspan, k, cp, cU, basecv);

	int suspan = findSpan(sn, sp, u, sU);
	std::vector<double> basesu;
	basisFuns(suspan, u, sp, sU, basesu);
	int svspan = findSpan(sm, sq, v, sV);
	std::vector<double> basesv;
	basisFuns(svspan, v, sq, sV, basesv);
	
	double x, y, z, w;
	Point4d Cw;
	for (int i = 0; i <= cp; i++) {
		Cw.x = Cw.x + basecv[i] * cP[cuspan - cp + i].x * curveWeight[cuspan - cp + i];
		Cw.y = Cw.y + basecv[i] * cP[cuspan - cp + i].y * curveWeight[cuspan - cp + i];
		Cw.z = Cw.z + basecv[i] * cP[cuspan - cp + i].z * curveWeight[cuspan - cp + i];
		Cw.w = Cw.w + basecv[i] * curveWeight[cuspan - cp + i];// 这里体现了分母的计算
	}
	Point3d C;
	// 计算 下面的 w
	if (Cw.w == 0) {
		std::cout << "Cw.w== 0 " << Cw.w << std::endl;
		exit(0);
	}
	C.x = Cw.x / Cw.w; C.y = Cw.y / Cw.w; C.z = Cw.z / Cw.w;
	//----------------面的计算--------------------
	std::vector<Point4d> temp;
	for (int l = 0; l <= sq; l++) {
		Point4d tmp;
		for (int k = 0; k <= sp; k++) {
			tmp.x = tmp.x + basesu[k] * sP[suspan - sp + k][svspan - sq + l].x * surfaceWeight[suspan - sp + k][svspan - sq + 1];
			tmp.y = tmp.y + basesu[k] * sP[suspan - sp + k][svspan - sq + l].y * surfaceWeight[suspan - sp + k][svspan - sq + 1];
			tmp.z = tmp.z + basesu[k] * sP[suspan - sp + k][svspan - sq + l].z * surfaceWeight[suspan - sp + k][svspan - sq + 1];
			tmp.w = tmp.w + basesu[k] * surfaceWeight[suspan - sp + k][svspan - sq + l];
		}
		temp.push_back(tmp);
	}
	Point4d Sw;
	for (int l = 0; l <= sq; l++) {
		Sw.x = Sw.x + basesv[l] * temp[l].x;
		Sw.y = Sw.y + basesv[l] * temp[l].y;
		Sw.z = Sw.z + basesv[l] * temp[l].z;
		Sw.w = Sw.w + basesv[l] * temp[l].w;
	}
	if (Sw.w == 0) {
		std::cout << "ERROR Sw.w == 0" << std::endl;
	}
	S.x = Sw.x / Sw.w;
	S.y = Sw.y / Sw.w;
	S.z = Sw.z / Sw.w;
	S.x = S.x + C.x;
	S.y = S.y + C.y;
	S.z = S.z + C.z;
}



/**************************************************
@brief   : 简单的讲生成的点加入到曲线上
@author  : lee
@input   ：str  文件路径
@output  ：none
@time    : none
**************************************************/
void SweptVolume::genPoints(std::string str) {
	jsonReader(str);
	std::cout << "[DEBUG]genPoint" << std::endl;

	SVCfg.volumePoints.clear();//逻辑上会递归清空
	for (int i = 0; i < SVCfg.curvaturePointsNumber; i++) {
		double k = i * 1.0 * (SVCfg.CU_vector[SVCfg.CU_vector.size() - 1] - SVCfg.CU_vector[0]) / SVCfg.curvaturePointsNumber;
		std::vector<std::vector<Point3d>> tmp;
		for (int j = 0; j < SVCfg.surfacePointsNumberRow; j++) {
			double u = j * 1.0 * (SVCfg.SU_vector[SVCfg.SU_vector.size() - 1] - SVCfg.SU_vector[0]) / SVCfg.surfacePointsNumberRow;//均匀分成多少个点再所有的节点上
			std::vector<Point3d> temp;
			for (int t = 0; t < SVCfg.surfacePointsNumberCol; t++) {
				double v = t * 1.0 * (SVCfg.SV_vector[SVCfg.SV_vector.size() - 1] - SVCfg.SV_vector[0]) / SVCfg.surfacePointsNumberCol;//均匀分成多少个点再所有的节点上
				Point3d S;
				volumePoint(SVCfg.curveCtrlPoints.size() - 1, SVCfg.CP_Power, SVCfg.CU_vector, SVCfg.curveWeight, SVCfg.curveCtrlPoints, k, 
					SVCfg.surfaceCtrlPoints.size()-1, SVCfg.SP_Power, SVCfg.SU_vector, SVCfg.surfaceCtrlPoints[0].size() - 1, SVCfg.SQ_Power,
					SVCfg.SV_vector, SVCfg.surfaceCtrlPoints, u, v, SVCfg.surfaceWeight, S);
				temp.push_back(S);
			}
			tmp.push_back(temp);
		}
		SVCfg.volumePoints.push_back(tmp);
	}
}



/**************************************************
@brief   : 从文件中读取相应的参数
@author  : lee
@input   ：文件名(完整路径)/或空 方便调试
@output  ：none
@time    : none
**************************************************/
void SweptVolume::jsonReader(std::string fileName) {
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
		SVCfg.name = root["name"].asString();

		SVCfg.CP_Power = root["CP_Power"].asInt();
		SVCfg.curvaturePointsNumber = root["curvaturePointsNumber"].asInt();
		Json::Value curveCtrlPoints = root["curveCtrlPoints"];
		for (int i = 0; i < curveCtrlPoints.size(); i++) {
			Point3d p(0, 0, 0);
			p.x = (curveCtrlPoints[i][0].asDouble());
			p.y = (curveCtrlPoints[i][1].asDouble());
			p.z = (curveCtrlPoints[i][2].asDouble());
			SVCfg.curveCtrlPoints.push_back(p);
		}
		Json::Value vectors = root["CU_vector"];
		for (int i = 0; i < vectors.size(); i++) {
			SVCfg.CU_vector.push_back(vectors[i].asDouble());
		}
		Json::Value curveWeight = root["curveWeight"];
		for (int i = 0; i < curveWeight.size(); i++) {
			SVCfg.curveWeight.push_back(curveWeight[i].asDouble());
		}
		//--------------------------------------------------------
		SVCfg.SP_Power = root["SP_Power"].asInt();
		SVCfg.SQ_Power = root["SQ_Power"].asInt();
		SVCfg.surfacePointsNumberCol = root["surfacePointsNumberCol"].asInt();
		SVCfg.surfacePointsNumberRow = root["surfacePointsNumberRow"].asInt();
		Json::Value surfaceCtrlPoints = root["surfaceCtrlPoints"];
		for (int i = 0; i < surfaceCtrlPoints.size(); i++) {
			std::vector<Point3d> temp;
			for (int j = 0; j < surfaceCtrlPoints[0].size(); j++) {
				Point3d p(0, 0, 0);
				p.x = (surfaceCtrlPoints[i][j][0].asDouble());
				p.y = (surfaceCtrlPoints[i][j][1].asDouble());
				p.z = (surfaceCtrlPoints[i][j][2].asDouble());
				temp.push_back(p);
			}
			SVCfg.surfaceCtrlPoints.push_back(temp);
		}
		Json::Value surfaceWeight = root["surfaceWeight"];
		for (int i = 0; i < surfaceWeight.size(); i++) {
			std::vector<double> temp;
			for (int j = 0; j < surfaceWeight[0].size(); j++) {
				double tmp = surfaceWeight[i][j].asDouble();
				temp.push_back(tmp);
			}
			SVCfg.surfaceWeight.push_back(temp);
		}
		Json::Value SU_vector = root["SU_vector"];
		for (int i = 0; i < SU_vector.size(); i++) {
			SVCfg.SU_vector.push_back(SU_vector[i].asDouble());
		}
		Json::Value SV_vector = root["SV_vector"];
		for (int i = 0; i < SV_vector.size(); i++) {
			SVCfg.SV_vector.push_back(SV_vector[i].asDouble());
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
void SweptVolume::initializeGL() {
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
void SweptVolume::resizeGL(int width, int height)
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
void SweptVolume::paintGL() {
	//genPoints("normal");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// 清除屏幕和深度缓存
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//平移

	glRotated(rotationX, 1.0, 0.0, 0.0);
	glRotated(rotationY, 0.0, 1.0, 0.0);
	glRotated(rotationZ, 0.0, 0.0, 1.0);//旋转
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
	//由于显示了内部的点，看不出来是否正确，不显示内部的连线
	for (int t = 0; t < SVCfg.volumePoints.size(); t++) {
		for (int i = 0; i < SVCfg.volumePoints[t].size(); i++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int j = 0; j < SVCfg.volumePoints[t][0].size(); j++) {
				glVertex3d(SVCfg.volumePoints[t][i][j].x, SVCfg.volumePoints[t][i][j].y, SVCfg.volumePoints[t][i][j].z);
			}
			glEnd();
		}
	}
	for (int t = 0; t < SVCfg.volumePoints.size(); t++) {
		for (int i = 0; i < SVCfg.volumePoints[t][0].size(); i++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int j = 0; j < SVCfg.volumePoints[t].size(); j++) {
				glVertex3d(SVCfg.volumePoints[t][j][i].x, SVCfg.volumePoints[t][j][i].y, SVCfg.volumePoints[t][j][i].z);
			}
			glEnd();
		}
	}
	// 画出垂直的线条
	for (int i = 0; i < SVCfg.volumePoints[0].size(); i++) {
		for (int j = 0; j < SVCfg.volumePoints[0][0].size(); j++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int t = 0; t < SVCfg.volumePoints.size(); t++) {
				glVertex3d(SVCfg.volumePoints[t][i][j].x, SVCfg.volumePoints[t][i][j].y, SVCfg.volumePoints[t][i][j].z);
			}
			glEnd();
		}
	}
	/*for (int t = 0; t < SVCfg.volumePoints.size(); t++) {
		for (int i = 0; i < SVCfg.volumePoints[t].size(); i++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int j = 0; j < SVCfg.volumePoints[t][0].size(); j++) {
				if(j==0 || i == 0 || t == 0 || i == SVCfg.volumePoints[t].size()-1 || j == SVCfg.volumePoints[t][0].size()-1 || t == SVCfg.volumePoints.size()-1)
					glVertex3d(SVCfg.volumePoints[t][i][j].x, SVCfg.volumePoints[t][i][j].y, SVCfg.volumePoints[t][i][j].z);
			}
			glEnd();
		}
	}
	for (int t = 0; t < SVCfg.volumePoints.size(); t++) {
		for (int i = 0; i < SVCfg.volumePoints[t][0].size(); i++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int j = 0; j < SVCfg.volumePoints[t].size(); j++) {
				if (j == 0 || i == 0 || t == 0 || i == SVCfg.volumePoints[t][0].size() - 1 || j == SVCfg.volumePoints[t].size() - 1 || t == SVCfg.volumePoints.size() - 1)
					glVertex3d(SVCfg.volumePoints[t][j][i].x, SVCfg.volumePoints[t][j][i].y, SVCfg.volumePoints[t][j][i].z);
			}
			glEnd();
		}
	}
	for (int i = 0; i < SVCfg.volumePoints[0].size(); i++) {
		for (int j = 0; j < SVCfg.volumePoints[0][0].size(); j++) {
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 1.0);
			for (int t = 0; t < SVCfg.volumePoints.size(); t++) {
				if (j == 0 || i == 0 || t == 0 || i == SVCfg.volumePoints[0].size() - 1 || j == SVCfg.volumePoints[0][0].size() - 1 || t == SVCfg.volumePoints.size() - 1)
					glVertex3d(SVCfg.volumePoints[t][i][j].x, SVCfg.volumePoints[t][i][j].y, SVCfg.volumePoints[t][i][j].z);
			}
			glEnd();
		}
	}*/
}



/**************************************************
@brief   : 按键按下事件
@author  : lee
@input   ：e  事件
@output  ：none
@time    : none
**************************************************/
void SweptVolume::mousePressEvent(QMouseEvent *e) {
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
void SweptVolume::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : none
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/
void SweptVolume::mouseMoveEvent(QMouseEvent *e) {
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
void SweptVolume::exportOBJ() {
	MyMesh mesh;
	int z = SVCfg.volumePoints.size();
	int x = SVCfg.volumePoints[0].size();
	int y = SVCfg.volumePoints[0][0].size();
	int faceDir[][2] = { {0, z - 1},{0, x - 1},{0, y - 1} };
	
	for (int t = 0; t < 1; t++) {
		std::vector<std::vector<Point3d>> v;//底面
		for (int i = 0; i < x; i++) {
			std::vector<Point3d> tmp;
			for (int j = 0; j < y; j++) {
				Point3d p(SVCfg.volumePoints[faceDir[0][t]][i][j].x, SVCfg.volumePoints[faceDir[0][t]][i][j].y, SVCfg.volumePoints[faceDir[0][t]][i][j].z);
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
				Point3d p(SVCfg.volumePoints[faceDir[0][t]][i][j].x, SVCfg.volumePoints[faceDir[0][t]][i][j].y, SVCfg.volumePoints[faceDir[0][t]][i][j].z);
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
				Point3d p(SVCfg.volumePoints[j][i][faceDir[2][t]].x, SVCfg.volumePoints[j][i][faceDir[2][t]].y, SVCfg.volumePoints[j][i][faceDir[2][t]].z);
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
				Point3d p(SVCfg.volumePoints[j][i][faceDir[2][t]].x, SVCfg.volumePoints[j][i][faceDir[2][t]].y, SVCfg.volumePoints[j][i][faceDir[2][t]].z);
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
				Point3d p(SVCfg.volumePoints[j][faceDir[1][t]][i].x, SVCfg.volumePoints[j][faceDir[1][t]][i].y, SVCfg.volumePoints[j][faceDir[1][t]][i].z);
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
				Point3d p(SVCfg.volumePoints[j][faceDir[1][t]][i].x, SVCfg.volumePoints[j][faceDir[1][t]][i].y, SVCfg.volumePoints[j][faceDir[1][t]][i].z);
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
void SweptVolume::genOneFaceMesh(MyMesh &mesh, int m, int n, const std::vector<std::vector<Point3d>> &v) {
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
void SweptVolume::genOtherFaceMesh(MyMesh &mesh, int m, int n, const std::vector<std::vector<Point3d>> &v) {
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
