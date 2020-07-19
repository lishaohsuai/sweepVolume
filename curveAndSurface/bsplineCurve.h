#ifndef __BSPLINECURVE_H__
#define __BSPLINECURVE_H__
#include "Strategy.h"
#include <QMouseEvent>
struct BsplineCurveConfig {
	std::string name;//名称
	std::vector<QPointF> ctrlPoints;//控制顶点
	std::vector<QPointF> curvaturePoints;//曲线上的点
	int curvaturePointsNumber;//曲线向的点的个数
	int X_Power;// p 阶次
	std::vector<double> U_vector;
};

class BsplineCurve :public QWidget, public Strategy {
	Q_OBJECT
public:
	BsplineCurve(QWidget *parent = 0);
	~BsplineCurve();
	void jsonReader(std::string);
	void genPoints(std::string);
	int findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U);
	void basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base);
	void curvePoint(int n, int p, const std::vector<double>&U, std::vector<QPointF>&P, double u, QPointF &C);
	void resizeGL(int width, int height);
protected:
	void paintEvent(QPaintEvent *);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);

private:
	BsplineCurveConfig bsConfig;
	int flag;//判断选中的是哪一个控制点
	int precision;//鼠标选择点的精度
};


#endif
