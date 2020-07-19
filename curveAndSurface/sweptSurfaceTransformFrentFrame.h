#ifndef __SWEPTSURFACETRANSFORMFRENTFRAME_H__
#define __SWEPTSURFACETRANSFORMFRENTFRAME_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>
#include "SweptSurfaceAbstractPlanar.h"
#include "SweptSurfaceAbstractFrentFrame.h"
#include "bsplineSurface.h"
#include "sweptSurface.h"
/**************************************************
@brief   : S(u,v) = T(v) + A(v)S(v)C(u) T(v) is 3d
@author  : lee
@input   ：none
@output  ：none
@time    : none
**************************************************/






class SweptSurfaceTransformFrentFrame :public QGLWidget, public Strategy {
	Q_OBJECT
public:
	SweptSurfaceTransformFrentFrame(QWidget *parent = 0);
	~SweptSurfaceTransformFrentFrame();
	void jsonReader(std::string);
	void genPoints(std::string);
	int findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U);
	void basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base);
	void curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u,
		Point3d &C, const std::vector<double> &weight);
	/*void surfacePoint(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V,
		const std::vector<Point3d> &P, const std::vector<Point3d> &Q, double u, double v, Point3d &S,
		const std::vector<double> &weight1, const std::vector<double> &weight2);*/
	void surfacePoint(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V,
		std::vector<std::vector<Point3d> >&P, double u, double v, Point3d &S);
	void resizeGL(int width, int height);
	void initializeGL();
protected:
	//void paintEvent(QPaintEvent *);
	void paintGL();//绘制opengl图形
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
private:
	NurbsDoubleCurveConfig nurbsDoCvCfg;// 两条 nurbs 曲线
	//NurbsCurveConfig nurbsCvCfg2;// 生成一个扫掠面
	int flag;//判断选中的是哪一个控制点
	QPointF lastPos;//因为是平面，所以只能获得（x,y)二维坐标
	double rotationX;
	double rotationY;
	double rotationZ;
	BSplineCurveAbstract T;
	BSplineCurveAbstract C;
	SweptSurfaceAbstractFrentFrame sweptEntity;
	BsplineSurfaceConfig3d cfg;
};


#endif
