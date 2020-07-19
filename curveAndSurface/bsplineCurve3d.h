#ifndef __BSPLINECURVE3D_H__
#define __BSPLINECURVE3D_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>

struct BsplineCurveConfig3d {
	std::string name;//名称
	std::vector<Point3d> ctrlPoints;//控制顶点
	std::vector<Point3d> curvaturePoints;//曲线上的点
	int curvaturePointsNumber;//曲线向的点的个数
	int X_Power;// p 阶次
	std::vector<double> U_vector;
};


class BsplineCurve3d :public QGLWidget, public Strategy {
	Q_OBJECT
public:
	BsplineCurve3d(QWidget *parent = 0);
	~BsplineCurve3d();
	void jsonReader(std::string);
	void genPoints(std::string);
	int findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U);
	void basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base);
	void curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u, Point3d &C);
	void resizeGL(int width, int height);
	void initializeGL();
protected:
	//void paintEvent(QPaintEvent *);
	void paintGL();//绘制opengl图形
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
private:
	BsplineCurveConfig3d bs3dConfig;
	int flag;//判断选中的是哪一个控制点
	QPointF lastPos;//因为是平面，所以只能获得（x,y)二维坐标
	double rotationX;
	double rotationY;
	double rotationZ;
};


#endif
