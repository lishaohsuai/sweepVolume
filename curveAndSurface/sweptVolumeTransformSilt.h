#ifndef __SweptVolumeTransformSILT_H__
#define __SweptVolumeTransformSILT_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>
#include "SweptVolumeAbstractSilt.h"
#include "bsplineSurface.h"
#include "sweptSurface.h"
#include "bsplineSurfaceAbstract.h"
#include "nurbsCurve.h"
#include "bsplineVolume.h"
/**************************************************
@brief   : S(u,v) = T(v) + A(v)S(v)C(u) T(v) is 3d �������Ų����ı仯
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/





typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;

class SweptVolumeTransformSilt :public QGLWidget, public Strategy {
	Q_OBJECT
public:
	SweptVolumeTransformSilt(QWidget *parent = 0);
	~SweptVolumeTransformSilt();
	void jsonReader(std::string);
	void genPoints(std::string);
	int findSpan(int n, int p, double u, const std::vector<double> &U);
	void basisFuns(int i, double u, int p, const std::vector<double> &U, std::vector<double> &N);
	void volumePoint(int n0, int p0, const std::vector<double>&U0, int n1, int p1, const std::vector<double> &U1,
		int n2, int p2, const std::vector<double> &U2, std::vector<std::vector<std::vector<Point3d>>>&P,
		double x, double y, double z, Point3d &V);
	void resizeGL(int width, int height);
	void initializeGL();
	void exportOBJ();
	void genOneFaceMesh(MyMesh &mesh, int m, int n, const std::vector<std::vector<Point3d>> &v);
	void genOtherFaceMesh(MyMesh &mesh, int m, int n, const std::vector<std::vector<Point3d>> &v);
	void curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u, Point3d &C);
	BSplineCurveAbstract jsonReaderScale(std::string ctrlPointsStr, std::string UVectorStr, std::string PStr, Json::Value &root);
protected:
	//void paintEvent(QPaintEvent *);
	void paintGL();//����openglͼ��
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);

private:
	NurbsDoubleCurveConfig nurbsDoCvCfg;// ���� nurbs ����
	NurbsCurveConfig scaleCurve;// scaleCuve
	int flag;//�ж�ѡ�е�����һ�����Ƶ�
	QPointF lastPos;//��Ϊ��ƽ�棬����ֻ�ܻ�ã�x,y)��ά����
	double rotationX;
	double rotationY;
	double rotationZ;
	double times;
	int TcurveCCPoints;
	BOOL Planar;
	BSplineCurveAbstract T;
	BSplineSurfaceAbstract Suv;
	BSplineCurveAbstract Sx;//��������
	BSplineCurveAbstract Sy;
	BSplineCurveAbstract Twist;//Ť������
	SweptVolumeAbstractSilt sweptEntity;
	BSplineVolumeConfig cfg;
	std::vector<Point3d> curvePoints;
};


#endif
