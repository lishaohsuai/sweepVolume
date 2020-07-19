#include "nurbsSurface.h"
#include <QFileDialog>
#include <fstream>
#include <QGLWidget>
#include <gl/glu.h>



/**************************************************
@brief   : ���캯�� �򵥵ĳ�ʼ��������ֵ
@author  : lee
@input   ��parent һ��ΪNULL
@output  ��none
@time    : none
**************************************************/
NurbsSurface::NurbsSurface(QWidget *parent) :
	Strategy(), QGLWidget(parent) {
	flag = -1;// û���κε�����¼�
	rotationX = 0;
	rotationY = 0;
	rotationZ = 0;
};



/**************************************************
@brief   : ���������ͷ������Դ
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/
NurbsSurface::~NurbsSurface() {}

/**************************************************
@brief   : �ҵ� u �����ڵ���������
@author  : lee
@input   : numOfCtlPoint ���Ƶ�ĸ���
		   order �������Ľ�
		   u ������һ���0��1�ϵĵ�
		   U �ڵ�����
@output  : �ڵ���������
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
@brief   : ����ǿյĻ�����
		   ��һ�Σ�p=0ʱ��ֻ��N[0]=1.0,��N0,0=1.0;p=1ʱ����N[0],N[1],��N0,1��N1,1;p=2ʱ����N[0],N[1],N[2],��N0,2  N1,2��N2,2
@author  : lee
@input   ��i �ڵ������ĵ�i������
		   u ���������ϵĵ�
		   order �״�
		   U �ڵ�����
@output  ��base �ӵ�0�׵�����Ľ׷��������������Ӧ�Ľ׺�����ֵ��һ����
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
@brief   : ���������ϵĵ�
@author  : lee
@input   : n+1 �п��Ƶ�ĸ���
		   p �״�
		   U �ڵ�����
			m+1 �ڵ�ĸ���
		   m+1 �п��Ƶ������
		   q ��һ���״�
		   V ��һ���ڵ�����
		   P  ���Ƶ����� ��ά����
		   u��v ��С���󣬴Ӷ����Ƴ���������
		   S ���ɵĵ������
		   weight ÿ�����Ƶ��Ȩ�� 
@output  ��none
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
@brief   : �򵥵Ľ����ɵĵ���뵽������
@author  : lee
@input   ��str  �ļ�·��
@output  ��none
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
		double u = i * 1.0 * (NurbsSfCfg.U_vector[NurbsSfCfg.U_vector.size() - 1] - NurbsSfCfg.U_vector[0]) / NurbsSfCfg.surfacePointsNumberRow;//���ȷֳɶ��ٸ��������еĽڵ���
		std::vector<Point3d> temp;
		for (int j = 0; j < NurbsSfCfg.surfacePointsNumberCol; j++) {
			double v = j * 1.0 * (NurbsSfCfg.V_vector[NurbsSfCfg.V_vector.size() - 1] - NurbsSfCfg.V_vector[0]) / NurbsSfCfg.surfacePointsNumberCol;//���ȷֳɶ��ٸ��������еĽڵ���
			Point3d S;
			surfacePoint(NurbsSfCfg.ctrlPoints.size() - 1, NurbsSfCfg.P_Power, NurbsSfCfg.U_vector, NurbsSfCfg.ctrlPoints[0].size() - 1, NurbsSfCfg.Q_Power
				, NurbsSfCfg.V_vector, NurbsSfCfg.ctrlPoints, u, v, S, NurbsSfCfg.weight);
			temp.push_back(S);
		}
		NurbsSfCfg.surfacePoints.push_back(temp);
	}
}



/**************************************************
@brief   : ���ļ��ж�ȡ��Ӧ�Ĳ���
@author  : lee
@input   ���ļ���(����·��)/��� �������
@output  ��none
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
@brief   : ��ʼ��opengl����
@author  : lee
@input   ��none
@output  ��none
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
@brief   : �ı䴰�ڵĴ�С
@author  : lee
@input   ��width ���
		   height �߶�
@output  ��none
@time    : none
**************************************************/
void NurbsSurface::resizeGL(int width, int height)
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


/**************************************************
@brief   : �Զ��ͱ��� ���ø�������ͼ��
@author  : none
@input   ��none
@output  ��none
@time    : none
**************************************************/
void NurbsSurface::paintGL() {
	//genPoints("normal");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// �����Ļ����Ȼ���
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(1.0, -4, -25.0);//ƽ��

	glRotated(rotationX, 1.0, 0.0, 0.0);
	glRotated(rotationY, 0.0, 1.0, 0.0);
	glRotated(rotationZ, 0.0, 0.0, 1.0);//��ת

	// �����Ƶ�
	glPointSize(8.0);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);
	for (int i = 0; i < NurbsSfCfg.ctrlPoints.size(); i++) {
		for (int j = 0; j < NurbsSfCfg.ctrlPoints[0].size(); j++) {
			glVertex3d(NurbsSfCfg.ctrlPoints[i][j].x, NurbsSfCfg.ctrlPoints[i][j].y, NurbsSfCfg.ctrlPoints[i][j].z);
		}
	}
	glEnd();


	// ������������
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
@brief   : ���������¼�
@author  : lee
@input   ��e  �¼�
@output  ��none
@time    : none
**************************************************/
void NurbsSurface::mousePressEvent(QMouseEvent *e) {
	lastPos = e->pos();
	flag = 1;
}


/**************************************************
@brief   : none
@author  : none
@input   ��none
@output  ��none
@time    : none
**************************************************/
void NurbsSurface::mouseReleaseEvent(QMouseEvent *e) {
	flag = -1;
}


/**************************************************
@brief   : none
@author  : none
@input   ��none
@output  ��none
@time    : none
**************************************************/
void NurbsSurface::mouseMoveEvent(QMouseEvent *e) {
	if (flag) {
		GLdouble dx = GLdouble(e->x() - lastPos.x()) / width();//QWidght �� ���
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
@brief   : ����obj�ļ�
@author  : lee
@input   ��none
@output  ��none
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
		vhandle[i] = new MyMesh::VertexHandle[n];//��ά������
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
			face_vhandles.push_back(vhandle[i + 1][j]);//����������
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