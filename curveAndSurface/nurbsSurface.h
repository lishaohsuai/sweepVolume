#ifndef __NURBSSURFACE_H__
#define __NURBSSURFACE_H__
#include "Strategy.h"
#include <QMouseEvent>
#include <QGLWidget>

struct NurbsSurfaceCfg {
	std::string name;//����
	std::vector<std::vector<Point3d>> ctrlPoints;//���ƶ���
	std::vector<std::vector<double>> weight;
	std::vector<std::vector<Point3d>> surfacePoints;//�����ϵĵ�
	int surfacePointsNumberRow;//�����ϵĵ�ĸ�����
	int surfacePointsNumberCol;//�����ϵĵ�ĸ�����
	int P_Power;// p �״�
	int Q_Power;// q �״�
	std::vector<double> U_vector;
	std::vector<double> V_vector;
};

class NurbsSurface :public QGLWidget, public Strategy {
	Q_OBJECT
public:
	NurbsSurface(QWidget *parent = 0);
	~NurbsSurface();
	void jsonReader(std::string);
	void genPoints(std::string);
	int findSpan(int numOfCtlPoint, int order, double u, const std::vector<double> &U);
	void basisFuns(int i, double u, int order, const std::vector<double> &U, std::vector<double> &base);
	void surfacePoint(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V,
		std::vector<std::vector<Point3d> >&P, double u, double v, Point3d &S, const std::vector<std::vector<double> > weight);
	void resizeGL(int width, int height);
	void initializeGL();
	void exportOBJ();
protected:
	//void paintEvent(QPaintEvent *);
	void paintGL();//����openglͼ��
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
private:
	NurbsSurfaceCfg NurbsSfCfg;
	int flag;//�ж�ѡ�е�����һ�����Ƶ�
	QPointF lastPos;//��Ϊ��ƽ�棬����ֻ�ܻ�ã�x,y)��ά����
	double rotationX;
	double rotationY;
	double rotationZ;
};

#endif //

