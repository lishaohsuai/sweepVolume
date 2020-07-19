#include "bsplineSurfaceAbstract.h" 


/**************************************************
@brief   : 对于Suv的一阶u导
@author  : lee
@input   ：u,v
@output  ：向量  切向量
@time    : none
**************************************************/
vector3d BSplineSurfaceAbstract::getUFirstDerivative(double u, double v) {
	return getUDeriOfBSplineSurface(p, U, q, V, controlPts, u, v);
}

/**************************************************
@brief   : 对于Suv的一阶v导
@author  : lee
@input   ：u,v
@output  ：向量  切向量
@time    : none
**************************************************/
vector3d BSplineSurfaceAbstract::getVFirstDerivative(double u, double v) {
	return getVDeriOfBSplineSurface(p, U, q, V, controlPts, u, v);
}


Point3d BSplineSurfaceAbstract::getPointAt(double u, double v) {
	return surfacePoint(n, p, U, m, q, V, controlPts, u, v);
}

BSplineSurfaceAbstract BSplineSurfaceAbstract::getUDeriBSplineSurface() {
	return getUDeriBSpline(p, U, q, V, controlPts);// 生成导数曲线？？
}

BSplineSurfaceAbstract BSplineSurfaceAbstract::getVDeriBSplineSurface() {
	return getVDeriBSpline(p, U, q, V, controlPts);// 生成导数曲线？？
}


/**************************************************
@brief   : 生成曲面上的点  公式法
@author  : lee
@input   : n+1 控制点的个数
		   p 阶次
		   U 节点向量
			m+1 节点的个数
		   m+1 另一个控制点的个数
		   q 另一个阶次
		   V 另一个节点向量
		   P  控制点数组 二维数组
		   u，v 从小到大，从而绘制出整个曲线
		   S 生成的点的坐标
@output  ：none
@time    : none
**************************************************/
Point3d surfacePoint(int n, int p, const std::vector<double>&U, int m, int q, const std::vector<double> &V,
	std::vector<std::vector<Point3d> >&P, double u, double v) {
	int uspan = findSpan(n, p, u, U);
	std::vector<double> baseu;
	basisFuns(uspan, u, p, U, baseu);
	std::vector<double> basev;
	int vspan = findSpan(m, q, v, V);
	basisFuns(vspan, v, q, V, basev);
	int uind = uspan - p;
	Point3d S(0, 0, 0);
	for (int l = 0; l <= q; l++) {
		Point3d temp(0, 0, 0);
		int vind = vspan - q + l;
		for (int k = 0; k <= p; k++) {
			temp.x = temp.x + baseu[k] * P[uind + k][vind].x;
			temp.y = temp.y + baseu[k] * P[uind + k][vind].y;
			temp.z = temp.z + baseu[k] * P[uind + k][vind].z;
		}
		S.x = S.x + basev[l] * temp.x;
		S.y = S.y + basev[l] * temp.y;
		S.z = S.z + basev[l] * temp.z;
	}
	return S;
}


/**************************************************
@brief   : 得到关于曲面的关于u的一阶导向量
@author  : lee
@input   ：曲面的
		   p
		   U
		   q
		   V
		   ctrl_pts
		   u
		   v
@output  ：向量
@time    : none
**************************************************/
vector3d getUDeriOfBSplineSurface(int p, std::vector<double> U, int q, std::vector<double> V, std::vector<std::vector<Point3d>> ctrl_pts, double u, double v)
{
	int n = ctrl_pts.size() - 1;

	int pd = p - 1;
	int nd = n - 1;

	int m = ctrl_pts[0].size() - 1;
	std::vector<double> Ud(U.size() - 2);
	copy(U.begin() + 1, U.end() - 1, Ud.begin());

	std::vector<std::vector<Point3d>> ctlptsD;
	for (int j = 0; j < m; j++) {
		std::vector<Point3d> line;
		for (int i = 0; i < n; i++)
			line.push_back((ctrl_pts[i + 1][j] - ctrl_pts[i][j])*p*(U[i + p + 1] - U[i + 1]));
		ctlptsD.push_back(line);
	}
	Point3d deri = surfacePoint(nd, pd, Ud, m, q, V, ctlptsD, u, v);
	vector3d deriv(deri.getX(), deri.getY(), deri.getZ());
	return deriv;
}

/**************************************************
@brief   : 得到关于曲面的关于v的一阶导向量
@author  : lee
@input   ：曲面的
		   p
		   U
		   q
		   V
		   ctrl_pts
		   u
		   v
@output  ：向量
@time    : none
**************************************************/
vector3d getVDeriOfBSplineSurface(int p, std::vector<double> U, int q, std::vector<double> V, std::vector<std::vector<Point3d>> ctrl_pts, double u, double v)
{
	int m = ctrl_pts[0].size() - 1;
	int n = ctrl_pts.size() - 1;

	int qd = q - 1;
	int md = m - 1;

	
	std::vector<double> Vd(V.size() - 2);
	copy(V.begin() + 1, V.end() - 1, Vd.begin());

	std::vector<std::vector<Point3d>> ctlptsD;
	for (int j = 0; j < m; j++) {
		std::vector<Point3d> line;
		for (int i = 0; i < n; i++)
			line.push_back((ctrl_pts[i][j+1] - ctrl_pts[i][j])*q*(V[j + q + 1] - V[j + 1]));
		ctlptsD.push_back(line);
	}
	Point3d deri = surfacePoint(n, p, U, md, qd, Vd, ctlptsD, u, v);
	vector3d deriv(deri.getX(), deri.getY(), deri.getZ());
	return deriv;
}



/**************************************************
@brief   : 返回一个曲面  原曲面关于u方向的一阶求导
@author  : lee
@input   ：曲面的
		   p
		   U
		   q
		   V
		   ctrl_pts
@output  ：新产生的曲线
@time    : none
**************************************************/
BSplineSurfaceAbstract getUDeriBSpline(int p, std::vector<double> U, int q, std::vector<double> V, std::vector<std::vector<Point3d>> ctrl_pts) {
	int n = ctrl_pts.size() - 1;

	int pd = p - 1;
	int nd = n - 1;

	int m = ctrl_pts[0].size() - 1;
	std::vector<double> Ud(U.size() - 2);
	copy(U.begin() + 1, U.end() - 1, Ud.begin());

	std::vector<std::vector<Point3d>> ctlptsD;
	for (int j = 0; j < m; j++) {
		std::vector<Point3d> line;
		for (int i = 0; i < n; i++)
			line.push_back((ctrl_pts[i + 1][j] - ctrl_pts[i][j])*p*(U[i + p + 1] - U[i + 1]));
		ctlptsD.push_back(line);
	}

	BSplineSurfaceAbstract surface(pd, nd,  q, m, Ud, V, ctlptsD);
	return surface;
}


/**************************************************
@brief   : 返回一个曲面  原曲面关于v方向的一阶求导
@author  : lee
@input   ：曲面的
		   p
		   U
		   q
		   V
		   ctrl_pts
@output  ：新产生的曲线
@time    : none
**************************************************/
BSplineSurfaceAbstract getVDeriBSpline(int p, std::vector<double> U, int q, std::vector<double> V, std::vector<std::vector<Point3d>> ctrl_pts) {
	int m = ctrl_pts[0].size() - 1;
	int n = ctrl_pts.size() - 1;

	int qd = q - 1;
	int md = m - 1;
	std::vector<double> Vd(V.size() - 2);
	copy(V.begin() + 1, V.end() - 1, Vd.begin());

	std::vector<std::vector<Point3d>> ctlptsD;
	for (int j = 0; j < m; j++) {
		std::vector<Point3d> line;
		for (int i = 0; i < n; i++)
			line.push_back((ctrl_pts[i][j + 1] - ctrl_pts[i][j])*q*(V[j + q + 1] - V[j + 1]));
		ctlptsD.push_back(line);
	}
	BSplineSurfaceAbstract surface(p, n, qd, md, U, Vd, ctlptsD);
	return surface;
}

//
//
///* The Nurbs book page 151
//Algorithm A5.1
//Input:  np,p,UP,Pw,u,k,s,r
//Output: nq,UQ,Qw
//*/
//
//void CurveKnotIns(int np, int p, std::vector<double> UP, std::vector<Point3d> Pw, double u, int s, int r, int &nq, std::vector<double> &UQ, std::vector<Point3d> &Qw)
//{
//	int mp = np + p + 1;
//	nq = np + r;
//	int k = findSpan(np, p, u, UP);  /* insert u span */
//
//	std::vector<Point3d> Rw(p + 1);
//
//	/*Load new knot vector*/
//	for (int i = 0; i <= k; i++) UQ[i] = UP[i];
//	for (int i = 1; i <= r; i++) UQ[k + i] = u;
//	for (int i = k + 1; i <= mp; i++) UQ[i + r] = UP[i];
//
//	/* Save unaltered control points*/
//	for (int i = 0; i <= k - p; i++) Qw[i] = Pw[i];
//	for (int i = k - s; i <= np; i++) Qw[i + r] = Pw[i];
//	for (int i = 0; i <= p - s; i++) Rw[i] = Pw[k - p + i];
//
//	/* Insert the knot r times*/
//	int L;
//	for (int j = 1; j <= r; j++)
//	{
//		L = k - p + j;
//		for (int i = 0; i <= p - j - s; i++)
//		{
//			double alpha = (u - UP[L + i]) / (UP[i + k + 1] - UP[L + i]);
//			Rw[i] = Rw[i + 1] * alpha + Rw[i] * (1 - alpha);
//		}
//		Qw[L] = Rw[0];
//		Qw[k + r - j - s] = Rw[p - j - s];
//	}
//
//	/* Load remaining control points */
//	for (int i = L + 1; i < k - s; i++)
//		Qw[i] = Rw[i - L];
//}
//
//BSplineCurveAbstract curvePointKnontInsertion(int n, int p, std::vector<double> U, std::vector<Point3d> ctrl_pts, double insertu, int s, int insertTimes)
//{
//	int nq;
//	std::vector<double>  UQ(U.size() + insertTimes);
//	std::vector<Point3d> Qw(n + 1 + insertTimes);
//	CurveKnotIns(n, p, U, ctrl_pts, insertu, s, insertTimes, nq, UQ, Qw);
//	std::vector<Point3d> newControlPts = Qw;
//
//	BSplineCurveAbstract curve(p, nq, UQ, Qw);
//
//	return curve;
//}