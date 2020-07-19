#ifndef __BSPLINESURFACE_H__
#define __BSPLINESURFACE_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>

struct BsplineSurfaceConfig3d {
	std::string name;//����
	std::vector<std::vector<Point3d>> ctrlPoints;//���ƶ���
	std::vector<std::vector<Point3d>> surfacePoints;//�����ϵĵ�
	int surfacePointsNumberRow;//�����ϵĵ�ĸ�����
	int surfacePointsNumberCol;//�����ϵĵ�ĸ�����
	int P_Power;// p �״�
	int Q_Power;// q �״�
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
	void paintGL();//����openglͼ��
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
private:
	BsplineSurfaceConfig3d bsSf3dCfg;
	int flag;//�ж�ѡ�е�����һ�����Ƶ�
	QPointF lastPos;//��Ϊ��ƽ�棬����ֻ�ܻ�ã�x,y)��ά����
	double rotationX;
	double rotationY;
	double rotationZ;
};

#endif //

