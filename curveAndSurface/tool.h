#ifndef __TOOL_H__
#define __TOOL_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>




class Tool :public QGLWidget, public Strategy {
	Q_OBJECT
public:
	Tool(QWidget *parent = 0);
	~Tool();
	void jsonReader(std::string);
	void genPoints(std::string);
	void flipWithMidLine(const std::vector<Point3d> &oldPoint, std::vector<Point3d> &newPoint);
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
	int flag;//判断选中的是哪一个控制点
	QPointF lastPos;//因为是平面，所以只能获得（x,y)二维坐标
	double rotationX;
	double rotationY;
	double rotationZ;
};


#endif
