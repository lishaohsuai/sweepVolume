#include "SweptSurfaceAbstractFrentFrame.h"


SweptSurfaceAbstractFrentFrame::SweptSurfaceAbstractFrentFrame(int p_, int q_, std::vector<double> knotU_, std::vector<double> knotV_, std::vector<std::vector<Point3d>> controlNet_)
{
	p = p_;
	q = q_;
	knotU = knotU_;
	knotV = knotV_;
	controlNet = controlNet_;
}

void SweptSurfaceAbstractFrentFrame::getTranslationalSweep(int p_, std::vector<double> knotU_, int q_, std::vector<double> knotV_, std::vector<Point3d> Unet, std::vector<Point3d> Vnet)
{
	p = p_;
	q = q_;
	knotU = knotU_;
	knotV = knotV_;

	/*
	   U: r = n + p + 1
	   V: s = m + q + 1
	*/
	int r = knotU.size() - 1;
	int s = knotV.size() - 1;
	int n = r - p - 1;
	int m = s - q - 1;
	int N = n + 1; // number of control points
	int M = m + 1; // number of control points

	controlNet.resize(N);
	for (int i = 0; i < N; i++)
		controlNet[i].resize(M);

	for (int i = 0; i < N; i++)
		for (int j = 0; j < M; j++)
			controlNet[i][j] = Unet[i] + Vnet[j];// 生成控制网格
}

void SweptSurfaceAbstractFrentFrame::outputSweptSur(int &p_, int &q_, std::vector<double> &knotU_, std::vector<double> &knotV_, std::vector<std::vector<Point3d>> &controlNet_)
{
	p_ = p;
	q_ = q;
	knotU_ = knotU;
	knotV_ = knotV;
	controlNet_ = controlNet;
}

void SweptSurfaceAbstractFrentFrame::getSweepSurface1(BSplineCurveAbstract T, BSplineCurveAbstract C, vector3d Bv, Matrix33 Sv, int K)
{
	p = C.getDegree();
	knotU = C.getKnotVec();
	q = T.getDegree();
	int ktv = T.getKnotVec().size();
	int nsect = K + 1;

	if (ktv <= nsect + q)
	{ // must refine T(v)'s knot vector
		int m = nsect + q - ktv + 1;
		// insert kont in T for m times
		insertKnots(T, m);
		knotV = T.getKnotVec();
	}
	else {
		knotV = T.getKnotVec();
		if (ktv > nsect + q + 1) // must increase number of instance of C(u)
			nsect = ktv - q - 1;
	}

	// get section v-s
	std::vector<double> vSections(nsect);
	vSections[0] = 0;// 最小的参数值
	vSections[nsect - 1] = 1;// 最大的参数值
	for (int k = 1; k < nsect - 1; k++)
	{
		vSections[k] = 0;
		for (int j = 1; j <= q; j++)
			vSections[k] += knotV[k + j];
		vSections[k] = vSections[k] / q;
	}

	// create transformed control points for each Section
	std::vector<std::vector<Point3d>> QTotalvFixed;  // vFixed for each row
	for (int k = 0; k < nsect; k++)
	{
		std::vector<Point3d> Q = C.getControlPts();
		for (int i = 0; i < Q.size(); i++)
			Q[i] = Sv * Q[i]; // scale every control point

		// for planar trajection curve, section curve facing y axis
		//vector3d yv = T.getFirstDerivative(vSections[k]).normalize();
		//vector3d xv = Bv.normalize();
		//vector3d zv = xv.crossp(yv);

		// for 3D trajectotion curve: use Frenet frame
		vector3d td1 = T.getFirstDerivative(vSections[k]).normalize();
		vector3d td2 = T.getDeriBSplineC().getFirstDerivative(vSections[k]).normalize();
		vector3d yv = td1;// 不知道为什么要乘 -1 后面做实验？？  Bv 没有在这里被使用
		vector3d xv = td1.crossp(td2); xv.normalize();
		vector3d zv = xv.crossp(yv);

		Point3d pTv = T.getPointAt(vSections[k]);
		Matrix33 Av(xv, yv, zv);

		for (int i = 0; i < Q.size(); i++)
			Q[i] = Av * Q[i] + pTv; // transform every control point


		QTotalvFixed.push_back(Q);
	}

	// get the tranverse of QTotalvFixed
	int rows = QTotalvFixed.size();
	int cols = QTotalvFixed[0].size();
	std::vector<std::vector<Point3d>> QTotaluFixed;  // uFixed for each row
	QTotaluFixed.resize(cols);
	for (int j = 0; j < cols; j++)
	{
		QTotaluFixed[j].resize(rows);
		for (int i = 0; i < rows; i++)
		{
			QTotaluFixed[j][i] = QTotalvFixed[i][j];
		}
	}

	int n = C.getControlPts().size();
	for (int i = 0; i < n; i++)
	{
		// TODO: interpolate across Q[0]....Q[nsect-1]
	}

	controlNet = QTotaluFixed;

}


/**************************************************
@brief   : 支持缩放
@author  : lee
@input   ：T 轨迹曲线
		   C 初始曲线
		   S 缩放曲线
		   Bv 垂直于T的向量
		   S 缩放向量
@output  ：none
@time    : none
**************************************************/
void SweptSurfaceAbstractFrentFrame::getSweepSurface2(BSplineCurveAbstract T, BSplineCurveAbstract C, vector3d Bv, BSplineCurveAbstract S, int K)
{
	p = C.getDegree();
	knotU = C.getKnotVec();
	q = T.getDegree();
	int ktv = T.getKnotVec().size();
	int nsect = K + 1;

	if (ktv <= nsect + q)
	{ // must refine T(v)'s knot vector
		int m = nsect + q - ktv + 1;
		// insert kont in T for m times
		insertKnots(T, m);
		knotV = T.getKnotVec();
	}
	else {
		knotV = T.getKnotVec();
		if (ktv > nsect + q + 1) // must increase number of instance of C(u)
			nsect = ktv - q - 1;
	}

	// get section v-s
	std::vector<double> vSections(nsect);
	vSections[0] = 0;// 最小的参数值
	vSections[nsect - 1] = 1;// 最大的参数值
	for (int k = 1; k < nsect - 1; k++)
	{
		vSections[k] = 0;
		for (int j = 1; j <= q; j++)
			vSections[k] += knotV[k + j];
		vSections[k] = vSections[k] / q;
	}

	// create transformed control points for each Section
	std::vector<std::vector<Point3d>> QTotalvFixed;  // vFixed for each row
	for (int k = 0; k < nsect; k++)
	{
		std::vector<Point3d> Q = C.getControlPts();
		Matrix33 Sv = Matrix33::getIdentityMatrix();
		for (int i = 0; i < Q.size(); i++) {
			Point3d tmp = S.getPointAt(vSections[k]);
			std::cout << "[DEBUG] vSections[k]" << vSections[k] << std::endl;
			Sv.setElement(0, 0, tmp.getX());
			Sv.setElement(1, 1, tmp.getY());
			Sv.setElement(2, 2, tmp.getZ());
			Q[i] = Sv * Q[i];
		}
			//Q[i] = S * Q[i]; // scale every control point  Sv的值是随vSections[k]变化而变化的。吃完饭测试

		// for planar trajection curve, section curve facing y axis
		//vector3d yv = T.getFirstDerivative(vSections[k]).normalize();
		//vector3d xv = Bv.normalize();
		//vector3d zv = xv.crossp(yv);

		// for 3D trajectotion curve: use Frenet frame
		vector3d td1 = T.getFirstDerivative(vSections[k]).normalize();
		vector3d td2 = T.getDeriBSplineC().getFirstDerivative(vSections[k]).normalize();
		vector3d yv = td1;// 不知道为什么要乘 -1 后面做实验？？  Bv 没有在这里被使用
		vector3d xv = td1.crossp(td2); xv.normalize();
		vector3d zv = xv.crossp(yv);

		Point3d pTv = T.getPointAt(vSections[k]);
		Matrix33 Av(xv, yv, zv);

		for (int i = 0; i < Q.size(); i++)
			Q[i] = Av * Q[i] + pTv; // transform every control point

		std::cout << "[DEBUG] QTotalvFixed " << k<< std::endl;
		QTotalvFixed.push_back(Q);
	}

	// get the tranverse of QTotalvFixed
	int rows = QTotalvFixed.size();
	int cols = QTotalvFixed[0].size();
	std::vector<std::vector<Point3d>> QTotaluFixed;  // uFixed for each row
	QTotaluFixed.resize(cols);
	for (int j = 0; j < cols; j++)
	{
		QTotaluFixed[j].resize(rows);
		for (int i = 0; i < rows; i++)
		{
			QTotaluFixed[j][i] = QTotalvFixed[i][j];
		}
	}

	int n = C.getControlPts().size();
	for (int i = 0; i < n; i++)
	{
		// TODO: interpolate across Q[0]....Q[nsect-1]
	}

	controlNet = QTotaluFixed;

}


/**************************************************
@brief   : 支持缩放 silt 显示曲面的效果
@author  : lee
@input   ：T 轨迹曲线
		   C 初始曲线
		   S 缩放曲线
		   //Bv 垂直于T的向量
		   S 缩放向量
@output  ：none
@time    : none
**************************************************/
void SweptSurfaceAbstractFrentFrame::getSweepSurfaceSilt(BSplineCurveAbstract T, BSplineCurveAbstract C,BSplineCurveAbstract S, int K){
	p = C.getDegree();
	knotU = C.getKnotVec();
	q = T.getDegree();
	int ktv = T.getKnotVec().size();
	int nsect = K + 1;

	if (ktv <= nsect + q)
	{ // must refine T(v)'s knot vector
		int m = nsect + q - ktv + 1;
		// insert kont in T for m times
		insertKnots(T, m);
		knotV = T.getKnotVec();
	}
	else {
		knotV = T.getKnotVec();
		if (ktv > nsect + q + 1) // must increase number of instance of C(u)
			nsect = ktv - q - 1;
	}

	// get section v-s
	std::vector<double> vSections(nsect);
	vSections[0] = 0;// 最小的参数值
	vSections[nsect - 1] = 1;// 最大的参数值
	for (int k = 1; k < nsect - 1; k++)
	{
		vSections[k] = 0;
		for (int j = 1; j <= q; j++)
			vSections[k] += knotV[k + j];
		vSections[k] = vSections[k] / q;
	}

	// create transformed control points for each Section
	std::vector<std::vector<Point3d>> QTotalvFixed;  // vFixed for each row
	vector3d bi;
	vector3d Ti;
	vector3d Bi;
	std::vector<vector3d> v;
	for (int k = 0; k < nsect; k++)
	{
		std::vector<Point3d> Q = C.getControlPts();
		double u = k * 1.0 * (T.getKnotVec()[T.getKnotVec().size() - 1] - T.getKnotVec()[0]) / nsect;//均匀分成多少个点再所有的节点上
		//Matrix33 Sv = Matrix33::getIdentityMatrix();
		//for (int i = 0; i < Q.size(); i++) {
			//Point3d tmp = S.getPointAt(vSections[k]);
			//std::cout << "[DEBUG] vSections[k]" << vSections[k] << std::endl;
			//Sv.setElement(0, 0, tmp.getX());
			//Sv.setElement(1, 1, tmp.getY());
			//Sv.setElement(2, 2, tmp.getZ());// 先不缩放
			//Q[i] = Sv * Q[i];
		//}
		//Q[i] = S * Q[i]; // scale every control point  Sv的值是随vSections[k]变化而变化的。吃完饭测试

		Ti = T.getFirstDerivative(u).normalize();
		if (k == 0) {
			vector3d B0;
			double delta = 0.001;
			while ((Ti.x == 0 && Ti.y == 0) ||
				(Ti.x == 0 && Ti.z == 0) ||
				(Ti.y == 0 && Ti.z == 0)) {
				Ti = T.getFirstDerivative(nsect + delta).normalize();
				delta += 0.001;
			}
			if (Ti.x != 0 && Ti.y != 0) {
				B0.x = Ti.y * -1; B0.y = Ti.x; B0.z = 0;
			}
			else if (Ti.x != 0 && Ti.z != 0) {
				B0.x = Ti.z * -1; B0.y = 0; B0.z = Ti.x;
			}
			else {
				B0.x = 0; B0.y = Ti.z * -1; B0.z = Ti.y;
			}
			//std::cout << " vertical? " << B0.x*Ti.x + B0.y*Ti.y + B0.z * Ti.z << std::endl;
			Bi = B0.normalize();
			bi = Bi - (Bi * Ti) * Ti;
		}
		else {	


			//double delta = 0.001;
			//while (Ti * Bi) {
			//	Ti = T.getFirstDerivative(vSections[k] + delta).normalize();
			//	delta += 0.001;
			//}
			//std::cout << " vertical2? " << Bi.x*Ti.x + Bi.y*Ti.y + Bi.z * Ti.z << std::endl;
			bi = Bi - Ti * (Bi.x*Ti.x + Bi.y*Ti.y+Bi.z*Ti.z) ;
			Bi = bi.normalize();
		}
		vector3d yv = T.getFirstDerivative(u).normalize();
		vector3d xv = Bi.normalize();//再silt方法中 Bv没有被使用
		vector3d zv = xv.crossp(yv);

		Point3d pTv = T.getPointAt(u);
		//Matrix33 Av(xv, yv, zv);
		Matrix33 Av(zv, yv, xv);// 垂直的

		for (int i = 0; i < Q.size(); i++)
			Q[i] = Av * Q[i] + pTv; // transform every control point

		//std::cout << "[DEBUG] QTotalvFixed " << k << std::endl;
		QTotalvFixed.push_back(Q);
	}

	// get the tranverse of QTotalvFixed
	int rows = QTotalvFixed.size();
	int cols = QTotalvFixed[0].size();
	std::vector<std::vector<Point3d>> QTotaluFixed;  // uFixed for each row
	QTotaluFixed.resize(cols);
	for (int j = 0; j < cols; j++)
	{
		QTotaluFixed[j].resize(rows);
		for (int i = 0; i < rows; i++)
		{
			QTotaluFixed[j][i] = QTotalvFixed[i][j];
		}
	}

	int n = C.getControlPts().size();
	for (int i = 0; i < n; i++)
	{
		// TODO: interpolate across Q[0]....Q[nsect-1]
	}

	controlNet = QTotaluFixed;

}

void SweptSurfaceAbstractFrentFrame::insertKnots(BSplineCurveAbstract& T, int insertTimes)
{
	// insesrt on T curve
	for (int i = 0; i < insertTimes; i++)
	{
		double insertu = getMiddlePointOfLongestVSpan(T.getKnotVec());
		T = curvePointKnontInsertion(T.getn(), T.getDegree(), T.getKnotVec(), T.getControlPts(), insertu, 0, 1);
	}
}

double SweptSurfaceAbstractFrentFrame::getMiddlePointOfLongestVSpan(std::vector<double> KnotVec)
{
	int indLongSpan = 0;

	double MaxSpan = 0;
	for (int i = 0; i < KnotVec.size() - 1; i++)
	{
		double currentSpan = KnotVec[i + 1] - KnotVec[i];
		if (currentSpan > MaxSpan)
		{
			indLongSpan = i;
			MaxSpan = currentSpan;
		}
	}

	return (KnotVec[indLongSpan + 1] + KnotVec[indLongSpan]) / 2.0;
}