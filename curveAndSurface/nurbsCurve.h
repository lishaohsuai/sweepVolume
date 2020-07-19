#ifndef __NURBSCURVE_H__
#define __NURBSCURVE_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>

struct NurbsCurveConfig {
	std::string name;//����
	std::vector<Point3d> ctrlPoints;//���ƶ���
	std::vector<Point3d> curvaturePoints;//�����ϵĵ�
	int curvaturePointsNumber;//������ĵ�ĸ���
	int X_Power;// p �״�
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
	void paintGL();//����openglͼ��
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
private:
	NurbsCurveConfig nurbsCvCfg;
	int flag;//�ж�ѡ�е�����һ�����Ƶ�
	QPointF lastPos;//��Ϊ��ƽ�棬����ֻ�ܻ�ã�x,y)��ά����
	double rotationX;
	double rotationY;
	double rotationZ;
};


#endif
