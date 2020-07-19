#ifndef __BSPLINECURVE3D_H__
#define __BSPLINECURVE3D_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>

struct BsplineCurveConfig3d {
	std::string name;//����
	std::vector<Point3d> ctrlPoints;//���ƶ���
	std::vector<Point3d> curvaturePoints;//�����ϵĵ�
	int curvaturePointsNumber;//������ĵ�ĸ���
	int X_Power;// p �״�
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
	void paintGL();//����openglͼ��
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
private:
	BsplineCurveConfig3d bs3dConfig;
	int flag;//�ж�ѡ�е�����һ�����Ƶ�
	QPointF lastPos;//��Ϊ��ƽ�棬����ֻ�ܻ�ã�x,y)��ά����
	double rotationX;
	double rotationY;
	double rotationZ;
};


#endif
