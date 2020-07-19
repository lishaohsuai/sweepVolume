#ifndef __SWEPTVOLUME_H__
#define __SWEPTVOLUME_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>


typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;

struct SweptVolumeConfig {
	std::string name;//名称
	// 轨迹线
	std::vector<Point3d> curveCtrlPoints;//控制顶点
	//std::vector<Point3d> curvaturePoints;//曲线上的点
	int curvaturePointsNumber;//曲线向的点的个数
	int CP_Power;// p 阶次
	std::vector<double> CU_vector;
	std::vector<double> curveWeight;
	// 平面
	std::vector<std::vector<Point3d>> surfaceCtrlPoints;//控制顶点
	std::vector<std::vector<double>> surfaceWeight;
	int surfacePointsNumberRow;//曲线上的点的个数行
	int surfacePointsNumberCol;//曲线上的点的个数列
	int SP_Power;// p 阶次
	int SQ_Power;// q 阶次
	std::vector<double> SU_vector;
	std::vector<double> SV_vector;
	std::vector<std::vector<std::vector<Point3d>>> volumePoints;//体上的点
};


class SweptVolume:public QGLWidget, public Strategy {
	Q_OBJECT
public:
	SweptVolume(QWidget *parent = 0);
	~SweptVolume();
	void jsonReader(std::string);
	void genPoints(std::string);
	int findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U);
	void basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base);
	void curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u,
		Point3d &C, const std::vector<double> &weight);
	void surfacePoint(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V,
		const std::vector<Point3d> &P, const std::vector<Point3d> &Q, double u, double v, Point3d &S,
		const std::vector<double> &weight1, const std::vector<double> &weight2);
	void volumePoint(int cn, int cp, const std::vector<double>&cU, const std::vector<double> &curveWeight, const std::vector<Point3d> &cP, double k,
		int sn, int sp, const std::vector<double> &sU,
		int sm, int sq, const std::vector<double> &sV,
		std::vector<std::vector<Point3d> >&sP, double u, double v, const std::vector<std::vector<double> > surfaceWeight, Point3d &S);
	void resizeGL(int width, int height);
	void initializeGL();
	void exportOBJ();
	void genOneFaceMesh(MyMesh &mesh, int m, int n, const std::vector<std::vector<Point3d>> &v);
	void genOtherFaceMesh(MyMesh &mesh, int m, int n, const std::vector<std::vector<Point3d>> &v);
protected:
	//void paintEvent(QPaintEvent *);
	void paintGL();//绘制opengl图形
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
private:
	SweptVolumeConfig SVCfg;// 两条 nurbs 曲线
	int flag;//判断选中的是哪一个控制点
	QPointF lastPos;//因为是平面，所以只能获得（x,y)二维坐标
	double rotationX;
	double rotationY;
	double rotationZ;
};


#endif
