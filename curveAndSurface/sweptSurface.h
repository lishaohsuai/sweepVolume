#ifndef __SWEPTSURFACE_H__
#define __SWEPTSURFACE_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>


/**************************************************
@brief   : S(u,v) = T(v) + C(u) 
@author  : lee
@input   ：none
@output  ：none
@time    : none
**************************************************/


struct NurbsDoubleCurveConfig {
	std::string name;//名称
	std::vector<Point3d> ctrlPoints1;//控制顶点
	std::vector<Point3d> curvaturePoints1;//曲线上的点
	std::vector<Point3d> ctrlPoints2;//控制顶点
	std::vector<Point3d> curvaturePoints2;//曲线上的点
	std::vector<std::vector<Point3d> > surfacePoints;
	int surfacePointsNum;//曲面上的点的个数
	int curvaturePointsNumber1;//曲线的点的个数
	int curvaturePointsNumber2;//曲线的点的个数
	int P_Power;// p 阶次
	int Q_Power;// q 阶次
	std::vector<double> U_vector;
	std::vector<double> V_vector;
	std::vector<double> weight1;
	std::vector<double> weight2;
};


class SweptSurface :public QGLWidget, public Strategy {
	Q_OBJECT
public:
	SweptSurface(QWidget *parent = 0);
	~SweptSurface();
	void jsonReader(std::string);
	void genPoints(std::string);
	int findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U);
	void basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base);
	void curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u,
		Point3d &C, const std::vector<double> &weight);
	void surfacePoint(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V,
		const std::vector<Point3d> &P, const std::vector<Point3d> &Q, double u, double v, Point3d &S,
		const std::vector<double> &weight1, const std::vector<double> &weight2);
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
};


#endif
