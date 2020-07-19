#ifndef __BSPLINECURVEABSTRACT_H__
#define __BSPLINECURVEABSTRACT_H__
#include <vector>
#include "Strategy.h"
#include "vector3d.h"
class BSplineCurveAbstract {
private:
	int p;
	int n;
	std::vector<double> U;
	std::vector<Point3d> controlPts;
	std::vector<double> controlPtsWeight;// 在这个方面先不使用？？  再看吧  这个作者还没有实现
public:
	BSplineCurveAbstract() { p = 0; n = 0; }
	//BSplineCurveAbstract(int p_, int n_, std::vector<double> U_, std::vector<Point3d> controlPts_, std::vector<double> controlPtsWeight_) :
		//p(p_), n(n_), U(U_), controlPts(controlPts_), controlPtsWeight(controlPtsWeight_) {}
	BSplineCurveAbstract(int p_, int n_, std::vector<double> U_, std::vector<Point3d> controlPts_) :
		p(p_), n(n_), U(U_), controlPts(controlPts_){}
	void set(const BSplineCurveAbstract & c) {
		p = c.p; n = c.n; U = c.U; controlPts = c.controlPts;
	}
	~BSplineCurveAbstract() {}// 暂时为空
	int getDegree() { return p; }
	int getn() { return n; }
	std::vector<double> getKnotVec() { return U; }
	std::vector<Point3d> getControlPts() { return controlPts; }

	vector3d getFirstDerivative(double u); 
	Point3d getPointAt(double u);
	BSplineCurveAbstract getDeriBSplineC();// 应该是得到B样条求导后的线条
};

/* The Nurbs book page 68
Algorithm A2.1
*/
int findSpan(int n, int p, double u, std::vector<double> U);

/* The Nurbs book page 70
Algorithm A2.2
*/

void basisFuns(int i, double u, int p, std::vector<double> U, std::vector<double> &N);

/* The Nurbs book page 82
Algorithm A3.1
*/
Point3d curvePoint(int n, int p, std::vector<double> U, std::vector<Point3d> ctrl_pts, double u);

/* The Nurbs book page 94
   formulation 3.4 -3.6
*/
vector3d getDeriOfBSplineCurve(int p, std::vector<double> U, std::vector<Point3d> ctrl_pts, double u);

/* The Nurbs book page 151
Algorithm A5.1
Input:  np,p,UP,Pw,u,k,s,r
Output: nq,UQ,Qw
*/
void CurveKnotIns(int np, int p, std::vector<double> UP, std::vector<Point3d> Pw, double u, int s, int r, int &nq, std::vector<double> &UQ, std::vector<Point3d> &Qw);

BSplineCurveAbstract curvePointKnontInsertion(int n, int p, std::vector<double> U, std::vector<Point3d> ctrl_pts, double insertu, int s, int insertTimes);

BSplineCurveAbstract getDeriBSplineCurve(int p, std::vector<double> U, std::vector<Point3d> ctrl_pts);




#endif
