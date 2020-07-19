#include "SweptSurfaceAbstractPlanar.h"


/*
The NURBS book
Algorithm A3.5
*/
Point3d getSurfacePoint(int n, int p, std::vector<double> U, int m, int q, std::vector<double> V, std::vector<std::vector<Point3d>> P, double u, double v)
{
	std::vector<double> Nu(p + 1);
	std::vector<double> Nv(q + 1);

	int uSpan = findSpan(n, p, u, U);
	basisFuns(uSpan, u, p, U, Nu);
	int vSpan = findSpan(m, q, v, V);
	basisFuns(vSpan, v, q, V, Nv);

	int uInd = uSpan - p;
	Point3d S;

	for (int l = 0; l <= q; l++)
	{
		Point3d temp;
		int vInd = vSpan - q + l;
		for (int k = 0; k <= p; k++)
			temp = temp + P[uInd + k][vInd] * Nu[k];
		S = S + temp * Nv[l];
	}

	return S;
}



SweptSurfaceAbstractPlanar::SweptSurfaceAbstractPlanar(int p_, int q_, std::vector<double> knotU_, std::vector<double> knotV_, std::vector<std::vector<Point3d>> controlNet_)
{
	p = p_;
	q = q_;
	knotU = knotU_;
	knotV = knotV_;
	controlNet = controlNet_;
}

void SweptSurfaceAbstractPlanar::getTranslationalSweep(int p_, std::vector<double> knotU_, int q_, std::vector<double> knotV_, std::vector<Point3d> Unet, std::vector<Point3d> Vnet)
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

void SweptSurfaceAbstractPlanar::outputSweptSur(int &p_, int &q_, std::vector<double> &knotU_, std::vector<double> &knotV_, std::vector<std::vector<Point3d>> &controlNet_)
{
	p_ = p;
	q_ = q;
	knotU_ = knotU;
	knotV_ = knotV;
	controlNet_ = controlNet;
}

void SweptSurfaceAbstractPlanar::getSweepSurface1(BSplineCurveAbstract T, BSplineCurveAbstract C, vector3d Bv, Matrix33 Sv, int K)
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
	else{
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
		vector3d yv = T.getFirstDerivative(vSections[k]).normalize();
		vector3d xv = Bv.normalize();
		vector3d zv = xv.crossp(yv);

		// for 3D trajectotion curve: use Frenet frame
		//vector3D td1 = T.getFirstDerivative(vSections[k]).normalize();
		//vector3D td2 = T.getDeriBSplineC().getFirstDerivative(vSections[k]).normalize();
		//vector3D yv = td1*(-1);
		//vector3D xv = td1.crossp(td2); xv.normalize();
		//vector3D zv = xv.crossp(yv);

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

void SweptSurfaceAbstractPlanar::insertKnots(BSplineCurveAbstract& T, int insertTimes)
{
	// insesrt on T curve
	for (int i = 0; i < insertTimes; i++)
	{
		double insertu = getMiddlePointOfLongestVSpan(T.getKnotVec());
		T = curvePointKnontInsertion(T.getn(), T.getDegree(), T.getKnotVec(), T.getControlPts(), insertu, 0, 1);
	}
}

double SweptSurfaceAbstractPlanar::getMiddlePointOfLongestVSpan(std::vector<double> KnotVec)
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