#ifndef __BEZIERVOLUME_H__
#define __BEZIERVOLUME_H__
#include "Strategy.h"
#include <QWidget.h>
#include <QGLWidget>
#include <fstream>
#include <QFileDialog>
#include <QPainter>
#include "debug.h"
#include <vector>
#include <gl/glu.h>
#include <QMouseEvent>
typedef struct BezierVolumeConfig {
	std::string name;
	std::vector<std::vector<std::vector<Point3d>>> ctrlPoints;//三个维度控制点
	int n;//n!
	int m;//m!
	int v;//v!
	int genPointNumX;//生成曲线的点的个数
	int genPointNumY;
	int genPointNumZ;
	std::vector<std::vector<std::vector<Point3d>>> volumePoints;
};


class BezierVolume : public QGLWidget, public Strategy {
	Q_OBJECT
public:
	BezierVolume(QWidget *parent = 0);
	~BezierVolume();

	void calcDeCasteljau2(const std::vector<std::vector<Point3d> > &P, int n, int m, double u0
		, double v0, Point3d &S);
	void calcDeCasteljau1(const std::vector<Point3d> &P, int n, double u, Point3d &_Q);
	void calcfactorialN(std::vector<int> &factorialN, int n);
	double calcBernstein(double u, int n, int i, std::vector<int> &factorialN);
	void jsonReader();
	void genPoints(std::string method);
	void paintGL();
	void initializeGL();
	void resizeGL(int width, int height);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mousePressEvent(QMouseEvent *event);
	Point3d changeto3d(double x, double y);
private:
	BezierVolumeConfig bezierVolumeConfig;
	GLfloat rotationX;
	GLfloat rotationY;
	GLfloat rotationZ;
	int ii;
	int jj;
	int flag;
	QPointF lastPos;
};




#endif