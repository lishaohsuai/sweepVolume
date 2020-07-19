#ifndef __NURBSCURVE_H__
#define __NURBSCURVE_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>

struct NurbsCurveConfig {
	std::string name;//名称
	std::vector<Point3d> ctrlPoints;//控制顶点
	std::vector<Point3d> curvaturePoints;//曲线上的点
	int curvaturePointsNumber;//曲线向的点的个数
	int X_Power;// p 阶次
	std::vector<double> U_vector;
	std::vector<double> weight;
};


class NurbsCurve :public QGLWidget, public Strategy {
	Q_OBJECT
public:
	NurbsCurve(QWidget *parent = 0);
	~NurbsCurve();
	void jsonReader(std::string);
	void genPoints(std::string);
	int findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U);
	void basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base);
	void curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u,
		Point3d &C, const std::vector<double> &weight);
	void resizeGL(int width, int height);
	void initializeGL();
protected:
	//void paintEvent(QPaintEvent *);
	void paintGL();//绘制opengl图形
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
private:
	NurbsCurveConfig nurbsCvCfg;
	int flag;//判断选中的是哪一个控制点
	QPointF lastPos;//因为是平面，所以只能获得（x,y)二维坐标
	double rotationX;
	double rotationY;
	double rotationZ;
};


#endif
