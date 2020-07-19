#ifndef __BSPLINESURFACE_H__
#define __BSPLINESURFACE_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>

struct BsplineSurfaceConfig3d {
	std::string name;//名称
	std::vector<std::vector<Point3d>> ctrlPoints;//控制顶点
	std::vector<std::vector<Point3d>> surfacePoints;//曲线上的点
	int surfacePointsNumberRow;//曲线上的点的个数行
	int surfacePointsNumberCol;//曲线上的点的个数列
	int P_Power;// p 阶次
	int Q_Power;// q 阶次
	std::vector<double> U_vector;
	std::vector<double> V_vector;
};

class BsplineSurface3d :public QGLWidget, public Strategy {
	Q_OBJECT
public:
	BsplineSurface3d(QWidget *parent = 0);
	~BsplineSurface3d();
	void jsonReader(std::string);
	void genPoints(std::string);
	int findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U);
	void basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base);
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
	BsplineSurfaceConfig3d bsSf3dCfg;
	int flag;//判断选中的是哪一个控制点
	QPointF lastPos;//因为是平面，所以只能获得（x,y)二维坐标
	double rotationX;
	double rotationY;
	double rotationZ;
};

#endif //

