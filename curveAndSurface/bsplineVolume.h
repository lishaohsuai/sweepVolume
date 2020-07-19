#ifndef __BSPLINEVOLUME_H__
#define __BSPLINEVOLUME_H__
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
typedef struct BSplineVolumeConfig {
	std::string name;
	std::vector<std::vector<std::vector<Point3d>>> ctrlPoints;//三个维度控制点
	std::vector<double> U0;
	std::vector<double> U1;
	std::vector<double> U2;
	int p0;// 对应 U
	int p1;// 对应 V
	int p2;// 对应 W
	int n0;
	int n1;
	int n2;
	int genPointNumX;//生成曲线的点的个数
	int genPointNumY;
	int genPointNumZ;
	std::vector<std::vector<std::vector<Point3d>>> volumePoints;
};


class BSplineVolume : public QGLWidget, public Strategy {
	Q_OBJECT
public:
	BSplineVolume(QWidget *parent = 0);
	~BSplineVolume();
	int findSpan(int n, int p, double u, const std::vector<double> &U);
	void basisFuns(int i, double u, int p, const std::vector<double> &U, std::vector<double> &N);
	void volumePoint(int n0, int p0, const std::vector<double>&U0, int n1, int p1, const std::vector<double> &U1,
		int n2, int p2, const std::vector<double> &U2, std::vector<std::vector<std::vector<Point3d>>>&P,
		double x, double y, double z, Point3d &V);
	void jsonReader();
	void genPoints(std::string method);
	void paintGL();
	void initializeGL();
	void resizeGL(int width, int height);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	Point3d changeto3d(double x, double y);
private:
	BSplineVolumeConfig bsplineVCfg;
	GLfloat rotationX;
	GLfloat rotationY;
	GLfloat rotationZ;
	GLfloat times;
	int ii;
	int jj;
	int flag;
	QPointF lastPos;

};




#endif