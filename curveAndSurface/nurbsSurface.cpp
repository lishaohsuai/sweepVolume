#include "nurbsSurface.h"
#include <QFileDialog>
#include <fstream>
#include <QGLWidget>
#include <gl/glu.h>



/**************************************************
@brief   : 构造函数 简单的初始化常量赋值
@author  : lee
@input   ：parent 一般为NULL
@output  ：none
@time    : none
**************************************************/
NurbsSurface::NurbsSurface(QWidget *parent) :
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
NurbsSurface::~NurbsSurface() {}

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
int NurbsSurface::findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U) {
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
void NurbsSurface::basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base) {
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
@brief   : 生成曲面上的点
@author  : lee
@input   : n+1 行控制点的个数
		   p 阶次
		   U 节点向量
			m+1 节点的个数
		   m+1 列控制点的列数
		   q 另一个阶次
		   V 另一个节点向量
		   P  控制点数组 二维数组
		   u，v 从小到大，从而绘制出整个曲线
		   S 生成的点的坐标
		   weight 每个控制点的权重 
@output  ：none
@time    : none
**************************************************/
void NurbsSurface::surfacePoint(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V,
	std::vector<std::vector<Point3d> >&P, double u, double v, Point3d &S, const std::vector<std::vector<double> > weight) {
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
			tmp.x = tmp.x + baseu[k] * P[uspan - p + k][vspan - q + l].x * weight[uspan - p + k][vspan - q + 1];
			tmp.y = tmp.y + baseu[k] * P[uspan - p + k][vspan - q + l].y * weight[uspan - p + k][vspan - q + 1];
			tmp.z = tmp.z + baseu[k] * P[uspan - p + k][vspan - q + l].z * weight[uspan - p + k][vspan - q + 1];
			tmp.w = tmp.w + baseu[k] * weight[uspan - p + k][vspan - q + l];
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
void NurbsSurface::genPoints(std::string str) {
	if (str != "")
		jsonReader(str);
	else
		jsonReader("");
	std::cout << "[DEBUG]genPoint" << std::endl;
	for (int i = 0; i < NurbsSfCfg.surfacePoints.size(); i++) {
		NurbsSfCfg.surfacePoints[i].clear();
	}
	NurbsSfCfg.surfacePoints.clear();

	for (int i = 0; i < NurbsSfCfg.surfacePointsNumberRow; i++) {
		double u = i * 1.0 * (NurbsSfCfg.U_vector[NurbsSfCfg.U_vector.size() - 1] - NurbsSfCfg.U_vector[0]) / NurbsSfCfg.surfacePointsNumberRow;//均匀分成多少个点再所有的节点上
		std::vector<Point3d> temp;
		for (int j = 0; j < NurbsSfCfg.surfacePointsNumberCol; j++) {
			double v = j * 1.0 * (NurbsSfCfg.V_vector[NurbsSfCfg.V_vector.size() - 1] - NurbsSfCfg.V_vector[0]) / NurbsSfCfg.surfacePointsNumberCol;//均匀分成多少个点再所有的节点上
			Point3d S;
			surfacePoint(NurbsSfCfg.ctrlPoints.size() - 1, NurbsSfCfg.P_Power, NurbsSfCfg.U_vector, NurbsSfCfg.ctrlPoints[0].size() - 1, NurbsSfCfg.Q_Power
				, NurbsSfCfg.V_vector, NurbsSfCfg.ctrlPoints, u, v, S, NurbsSfCfg.weight);
			temp.push_back(S);
		}
		NurbsSfCfg.surfacePoints.push_back(temp);
	}
}



/**************************************************
@brief   : 从文件中读取相应的参数
@author  : lee
@input   ：文件名(完整路径)/或空 方便调试
@output  ：none
@time    : none
**************************************************/
void NurbsSurface::jsonReader(std::string fileName) {
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
		NurbsSfCfg.name = root["name"].asString();
		NurbsSfCfg.P_Power = root["P_Power"].asInt();
		NurbsSfCfg.Q_Power = root["Q_Power"].asInt();
		NurbsSfCfg.surfacePointsNumberCol = root["surfacePointsNumberCol"].asInt();
		NurbsSfCfg.surfacePointsNumberRow = root["surfacePointsNumberRow"].asInt();
		Json::Value points = root["ctrlPoint"];
		for (int i = 0; i < points.size(); i++) {
			std::vector<Point3d> temp;
			for (int j = 0; j < points[0].size(); j++) {
				Point3d p(0, 0, 0);
				p.x = (points[i][j][0].asDouble());
				p.y = (points[i][j][1].asDouble());
				p.z = (points[i][j][2].asDouble());
				temp.push_back(p);
			}
			NurbsSfCfg.ctrlPoints.push_back(temp);
		}
		Json::Value weight = root["weight"];
		for (int i = 0; i < weight.size(); i++) {
			std::vector<double> temp;
			for (int j = 0; j < weight[0].size(); j++) {
				double tmp = weight[i][j].asDouble();
				temp.push_back(tmp);
			}
			NurbsSfCfg.weight.push_back(temp);
		}
		Json::Value vectorsU = root["U_vector"];
		for (int i = 0; i < vectorsU.size(); i++) {
			NurbsSfCfg.U_vector.push_back(vectorsU[i].asDouble());
		}
		Json::Value vectorsV = root["V_vector"];
		for (int i = 0; i < vectorsV.size(); i++) {
			NurbsSfCfg.V_vector.push_back(vectorsV[i].asDouble());
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
void NurbsSurface::initializeGL() {
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
void NurbsSurface::resizeGL(int width, int height)
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
void NurbsSurface::paintGL() {
	//genPoints("normal");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// 清除屏幕和深度缓存
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//平移

	glRotated(rotationX, 1.0, 0.0, 0.0);
	glRotated(rotationY, 0.0, 1.0, 0.0);
	glRotated(rotationZ, 0.0, 0.0, 1.0);//旋转

	// 画控制点
	glPointSize(8.0);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);
	for (int i = 0; i < NurbsSfCfg.ctrlPoints.size(); i++) {
		for (int j = 0; j < NurbsSfCfg.ctrlPoints[0].size(); j++) {
			glVertex3d(NurbsSfCfg.ctrlPoints[i][j].x, NurbsSfCfg.ctrlPoints[i][j].y, NurbsSfCfg.ctrlPoints[i][j].z);
		}
	}
	glEnd();


	// 画出网格曲线
	for (int i = 0; i < NurbsSfCfg.surfacePoints.size(); i++) {
		glBegin(GL_LINE_STRIP);
		glColor3f(1.0, 1.0, 1.0);
		for (int j = 0; j < NurbsSfCfg.surfacePoints[0].size(); j++) {
			glVertex3d(NurbsSfCfg.surfacePoints[i][j].x, NurbsSfCfg.surfacePoints[i][j].y, NurbsSfCfg.surfacePoints[i][j].z);
		}
		glEnd();
	}
	for (int i = 0; i < NurbsSfCfg.surfacePoints[0].size(); i++) {
		glBegin(GL_LINE_STRIP);
		glColor3f(1.0, 1.0, 1.0);
		for (int j = 0; j < NurbsSfCfg.surfacePoints.size(); j++) {
			glVertex3d(NurbsSfCfg.surfacePoints[j][i].x, NurbsSfCfg.surfacePoints[j][i].y, NurbsSfCfg.surfacePoints[j][i].z);
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
void NurbsSurface::mousePressEvent(QMouseEvent *e) {
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
void NurbsSurface::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : none
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/
void NurbsSurface::mouseMoveEvent(QMouseEvent *e) {
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
void NurbsSurface::exportOBJ() {
	typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;
	MyMesh mesh;
	int m, n;

	//cin >> m >> n;
	m = NurbsSfCfg.surfacePointsNumberRow;
	n = NurbsSfCfg.surfacePointsNumberCol;
	double length = 1.0;
	//NurbsSfCfg
	MyMesh::VertexHandle **vhandle = new MyMesh::VertexHandle*[m];
	for (int i = 0; i < m; i++) {
		vhandle[i] = new MyMesh::VertexHandle[n];//二维数组句柄
	}
	std::vector<MyMesh::VertexHandle>face_vhandles;
	double x = -m * length / 2;
	double y = -n * length / 2;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			vhandle[i][j] = mesh.add_vertex(MyMesh::Point(NurbsSfCfg.surfacePoints[i][j].x, NurbsSfCfg.surfacePoints[i][j].y, NurbsSfCfg.surfacePoints[i][j].z));
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

}