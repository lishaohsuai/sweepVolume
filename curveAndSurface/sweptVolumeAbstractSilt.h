#ifndef __SWEPTVOLUMEABSTRACTSILT_H__
#define __SWEPTVOLUMEABSTRACTSILT_H__
#include <Strategy.h>
#include "Matrix33.h"
#include "bsplineCurveAbstract.h"
#include "bsplineSurfaceAbstract.h"
/**************************************************
@brief   : ��Ĵ���ʽ
@author  : lee
@input   ��none
@output  ��none
@time    : none
**************************************************/
class SweptVolumeAbstractSilt {
private:
	int p0;
	int p1;
	int p2;
	std::vector<double> U0;
	std::vector<double> U1;
	std::vector<double> U2;
	std::vector<std::vector<std::vector<Point3d>>> controlNet;
public:
	SweptVolumeAbstractSilt() { p0 = p1 = p2 = 0; }
	SweptVolumeAbstractSilt(int p0_, int p1_, int p2_, std::vector<double> U0_, std::vector<double> U1_, std::vector<double> U2_, std::vector<std::vector<std::vector<Point3d>>> controlNet_);
	~SweptVolumeAbstractSilt() {};
	void outputSweptVolume(int &p0_, int &p1_, int &p2_, std::vector<double> &U0_, std::vector<double> &U1_, std::vector<double> &U2_, std::vector<std::vector<std::vector<Point3d>>> &controlNet_);
	//void clearup() { p0 = p1 = 0; knotU.clear(); knotV.clear(); controlNet.clear(); };
	void insertKnots(BSplineCurveAbstract& T, int insertTimes);
	double getMiddlePointOfLongestVSpan(std::vector<double> KnotVec);
	void getTranslationalSweep(int p_, std::vector<double> knotU_, int q_, std::vector<double> V_, std::vector<Point3d> Unet, std::vector<Point3d> Vnet);

	void getSweepVolumeSilt(BSplineCurveAbstract T, BSplineSurfaceAbstract Suv,
		BSplineCurveAbstract Sx, BSplineCurveAbstract Sy, BSplineCurveAbstract Twist, int K);
};


#endif
