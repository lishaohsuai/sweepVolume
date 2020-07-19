#ifndef __SWEPTVOLUME_H__
#define __SWEPTVOLUME_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>


typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;

struct SweptVolumeConfig {
	std::string name;//����
	// �켣��
	std::vector<Point3d> curveCtrlPoints;//���ƶ���
	//std::vector<Point3d> curvaturePoints;//�����ϵĵ�
	int curvaturePointsNumber;//������ĵ�ĸ���
	int CP_Power;// p �״�
	std::vector<double> CU_vector;
	std::vector<double> curveWeight;
	// ƽ��
	std::vector<std::vector<Point3d>> surfaceCtrlPoints;//���ƶ���
	std::vector<std::vector<double>> surfaceWeight;
	int surfacePointsNumberRow;//�����ϵĵ�ĸ�����
	int surfacePointsNumberCol;//�����ϵĵ�ĸ�����
	int SP_Power;// p �״�
	int SQ_Power;// q �״�
	std::vector<double> SU_vector;
	std::vector<double> SV_vector;
	std::vector<std::vector<std::vector<Point3d>>> volumePoints;//���ϵĵ�
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
	void paintGL();//����openglͼ��
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
private:
	SweptVolumeConfig SVCfg;// ���� nurbs ����
	int flag;//�ж�ѡ�е�����һ�����Ƶ�
	QPointF lastPos;//��Ϊ��ƽ�棬����ֻ�ܻ�ã�x,y)��ά����
	double rotationX;
	double rotationY;
	double rotationZ;
};


#endif
