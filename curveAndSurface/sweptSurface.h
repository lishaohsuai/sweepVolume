#ifndef __SWEPTSURFACE_H__
#define __SWEPTSURFACE_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>


/**************************************************
@brief   : S(u,v) = T(v) + C(u) 
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/


struct NurbsDoubleCurveConfig {
	std::string name;//����
	std::vector<Point3d> ctrlPoints1;//���ƶ���
	std::vector<Point3d> curvaturePoints1;//�����ϵĵ�
	std::vector<Point3d> ctrlPoints2;//���ƶ���
	std::vector<Point3d> curvaturePoints2;//�����ϵĵ�
	std::vector<std::vector<Point3d> > surfacePoints;
	int surfacePointsNum;//�����ϵĵ�ĸ���
	int curvaturePointsNumber1;//���ߵĵ�ĸ���
	int curvaturePointsNumber2;//���ߵĵ�ĸ���
	int P_Power;// p �״�
	int Q_Power;// q �״�
	std::vector<double> U_vector;
	std::vector<double> V_vector;
	std::vector<double> weight1;
	std::vector<double> weight2;
};


class SweptSurface :public QGLWidget, public Strategy {
	Q_OBJECT
public:
	SweptSurface(QWidget *parent = 0);
	~SweptSurface();
	void jsonReader(std::string);
	void genPoints(std::string);
	int findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U);
	void basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base);
	void curvePoint(int n, int p, const std::vector<double>&U, std::vector<Point3d>&P, double u,
		Point3d &C, const std::vector<double> &weight);
	void surfacePoint(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V,
		const std::vector<Point3d> &P, const std::vector<Point3d> &Q, double u, double v, Point3d &S,
		const std::vector<double> &weight1, const std::vector<double> &weight2);
	void resizeGL(int width, int height);
	void initializeGL();
protected:
	//void paintEvent(QPaintEvent *);
	void paintGL();//����openglͼ��
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
private:
	NurbsDoubleCurveConfig nurbsDoCvCfg;// ���� nurbs ����
	//NurbsCurveConfig nurbsCvCfg2;// ����һ��ɨ����
	int flag;//�ж�ѡ�е�����һ�����Ƶ�
	QPointF lastPos;//��Ϊ��ƽ�棬����ֻ�ܻ�ã�x,y)��ά����
	double rotationX;
	double rotationY;
	double rotationZ;
};


#endif
