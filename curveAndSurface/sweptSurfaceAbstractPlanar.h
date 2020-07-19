#ifndef __SweptSurfaceAbstractPlanar_H__
#define __SweptSurfaceAbstractPlanar_H__
#include <Strategy.h>
#include "Matrix33.h"
#include "bsplineCurveAbstract.h"

/**************************************************
@brief   : 平面T(v)的处理方式
@author  : none
@input   ：none
@output  ：none
@time    : none
**************************************************/

Point3d getSurfacePoint(int n, int p, std::vector<double> U, int m, int q, std::vector<double> V, std::vector<std::vector<Point3d>> P, double u, double v);
class SweptSurfaceAbstractPlanar{
private:
	int p;
	int q;
	std::vector<double> knotU;
	std::vector<double> knotV;
	std::vector<std::vector<Point3d>> controlNet;
public:
	SweptSurfaceAbstractPlanar() { p = q = 0; }
	SweptSurfaceAbstractPlanar(int p_, int q_, std::vector<double> knotU_, std::vector<double> knotV_, std::vector<std::vector<Point3d>> controlNet_);

	void outputSweptSur(int &p_, int &q_, std::vector<double> &knotU_, std::vector<double> &knotV_, std::vector<std::vector<Point3d>> &controlNet_);
	void clearup() { p = q = 0; knotU.clear(); knotV.clear(); controlNet.clear(); };
	void insertKnots(BSplineCurveAbstract& T, int insertTimes);
	double getMiddlePointOfLongestVSpan(std::vector<double> KnotVec);

	void getTranslationalSweep(int p_, std::vector<double> knotU_, int q_, std::vector<double> V_, std::vector<Point3d> Unet, std::vector<Point3d> Vnet);
	void getSweepSurface1(BSplineCurveAbstract T, BSplineCurveAbstract C, vector3d Bv, Matrix33 Sv, int K);
};


#endif
