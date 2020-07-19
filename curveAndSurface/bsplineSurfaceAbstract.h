#ifndef __BSPLINESURFACEABSTRACT_H__
#define __BSPLINESURFACEABSTRACT_H__
#include <vector>
#include "Strategy.h"
#include "vector3d.h"
#include "bsplineCurveAbstract.h"
class BSplineSurfaceAbstract {
private:
	int p;
	int q;
	int n;
	int m;
	std::vector<double> U;
	std::vector<double> V;
	std::vector<std::vector<Point3d>> controlPts;
	std::vector<double> controlPtsWeight;// 在这个方面先不使用？？  再看吧  这个作者还没有实现
	int surfacePointsNumberRow;//曲面上的点行
	int surfacePointsNumberCol;//曲面上的点列
public:
	BSplineSurfaceAbstract() { p = 0; n = 0; q = 0; m = 0; }
	BSplineSurfaceAbstract(int p_, int n_, int q_, int m_, std::vector<double> U_, std::vector<double> V_, std::vector<std::vector<Point3d>> controlPts_) :
		p(p_), n(n_),q(q_), m(m_),U(U_), V(V_), controlPts(controlPts_) {}
	void set(const BSplineSurfaceAbstract & s) {
		p = s.p; n = s.n; q = s.q; m = s.m;
		U = s.U; V = s.V; controlPts = s.controlPts;
	}
	~BSplineSurfaceAbstract() {}// 暂时为空
	int getDegreeP() { return p; }
	int getDegreeQ() { return q; }
	int getn() { return n; }
	int getm() { return m; }
	std::vector<double> getKnotVecU() { return U; }
	std::vector<double> getKnotVecV() { return V; }
	std::vector<std::vector<Point3d>> getControlPts() { return controlPts; }

	vector3d getUFirstDerivative(double u, double v);
	vector3d getVFirstDerivative(double u, double v);
	Point3d getPointAt(double u, double v);
	BSplineSurfaceAbstract getUDeriBSplineSurface();// 应该是得到B样条曲面求导后的曲面
	BSplineSurfaceAbstract getVDeriBSplineSurface();// 应该是得到B样条曲面求导后的曲面
};

Point3d surfacePoint(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V,
	std::vector<std::vector<Point3d> >&P, double u, double v);

vector3d getUDeriOfBSplineSurface(int p, std::vector<double> U, int q, std::vector<double> V, std::vector<std::vector<Point3d>> ctrl_pts, double u, double v);

vector3d getVDeriOfBSplineSurface(int p, std::vector<double> U, int q, std::vector<double> V, std::vector<std::vector<Point3d>> ctrl_pts, double u, double v);
//
///* The Nurbs book page 151
//Algorithm A5.1
//Input:  np,p,UP,Pw,u,k,s,r
//Output: nq,UQ,Qw
//*/
//void CurveKnotIns(int np, int p, std::vector<double> UP, std::vector<Point3d> Pw, double u, int s, int r, int &nq, std::vector<double> &UQ, std::vector<Point3d> &Qw);
//
//BSplineCurveAbstract curvePointKnontInsertion(int n, int p, std::vector<double> U, std::vector<Point3d> ctrl_pts, double insertu, int s, int insertTimes);
//
BSplineSurfaceAbstract getUDeriBSpline(int p, std::vector<double> U, int q, std::vector<double> V, std::vector<std::vector<Point3d>> ctrl_pts);
BSplineSurfaceAbstract getVDeriBSpline(int p, std::vector<double> U, int q, std::vector<double> V, std::vector<std::vector<Point3d>> ctrl_pts);




#endif
