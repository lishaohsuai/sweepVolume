#ifndef __BEZIERCURVE_H__
#define __BEZIERCURVE_H__
#include "Strategy.h"
#include <QWidget.h>
#include <fstream>
#include <QFileDialog>
#include <QPainter>
#include "debug.h"
typedef struct BezierCurveConfig {
	std::string name;
	std::vector<QPointF> ctrlPoints;
	int n;//n!
	int genPointNum;//生成曲线的点的个数
	std::vector<QPointF> curvaturePoints;
};
class BezierCurve : public QWidget,public Strategy{
	Q_OBJECT
public:
	BezierCurve(QWidget *parent = 0);
	~BezierCurve();
	void calcfactorialN(std::vector<int> &factorialN);
	double calcBernstein(double u, int n, int i, std::vector<int> &factorialN);
	QPointF calcBernsteinDeCasteljau(double u, int n, int i);
	void jsonReader();
	void genPoints(std::string method);
	//void draw();
	void paintEvent(QPaintEvent *);
private:
	BezierCurveConfig bezierCurveConfig;
};


#endif
