#ifndef __BEZIERSURFACE_H__
#define __BEZIERSURFACE_H__

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


typedef struct BezierSurfaceConfig {
	std::string name;
	std::vector<std::vector<Point3d> > ctrlPoints;
	int n;//n!
	int m;//m!
	int genPointNumX;//生成曲线的点的个数
	int genPointNumY;
	std::vector<std::vector<Point3d> > surfacePoints;
};
class BezierSurface : public QGLWidget, public Strategy {
	Q_OBJECT
public:
	BezierSurface(QWidget *parent = 0);
	~BezierSurface();

	void calcDeCasteljau2(const std::vector<std::vector<Point3d> > &P, int n, int m, double u0
		, double v0, Point3d &S);
	void calcDeCasteljau1(const std::vector<Point3d> &P, int n, double u, Point3d &_Q);
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
	BezierSurfaceConfig bezierSurfaceConfig;
	GLfloat rotationX;
	GLfloat rotationY;
	GLfloat rotationZ;
	int ii;
	int jj;
	int flag;
	QPointF lastPos;
};



#endif
