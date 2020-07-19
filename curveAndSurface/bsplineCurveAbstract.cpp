#include "bsplineCurveAbstract.h" 

vector3d BSplineCurveAbstract::getFirstDerivative(double u) {
	return getDeriOfBSplineCurve(p, U, controlPts, u);//对一个点的求导
}

Point3d BSplineCurveAbstract::getPointAt(double u) {
	return curvePoint(n, p, U, controlPts, u);
}

BSplineCurveAbstract BSplineCurveAbstract::getDeriBSplineC(){
	return getDeriBSplineCurve(p, U, controlPts);// 生成导数曲线？？
}

/* The Nurbs book page 68
Algorithm A2.1
*/
int findSpan(int n, int p, double u, std::vector<double> U)
{
	if (u >= U[n + 1]) return n;
	for (int i = p; i < n + 1; i++) {
		if (U[i] <= u && u < U[i + 1]) {
			return i;
		}
	}
}

/* The Nurbs book page 70
Algorithm A2.2
*/

void basisFuns(int i, double u, int p, std::vector<double> U, std::vector<double> &N)
{
	N.resize(p + 1);
	N[0] = 1.0;
	std::vector<double> left(p + 1);   // only the last p elements are used
	std::vector<double> right(p + 1);

	for (int j = 1; j <= p; j++)
	{
		left[j] = u - U[i + 1 - j];
		right[j] = U[i + j] - u;
		double saved = 0;

		for (int r = 0; r < j; r++)
		{
			double temp = N[r] / (right[r + 1] + left[j - r]);
			N[r] = saved + right[r + 1] * temp;
			saved = left[j - r] * temp;
		}
		N[j] = saved;
	}
}

/* 
*/

/**************************************************
@brief   : The Nurbs book page 82
		   Algorithm A3.1
		   求曲线上的一个点
@author  : lee
@input   ：原来曲线上的
		   n
		   p
		   U
		   ctrl_pts
		   u   一般是从0到1的浮点数
@output  ：u 所对应的点
@time    : none
**************************************************/
Point3d curvePoint(int n, int p, std::vector<double> U, std::vector<Point3d> ctrl_pts, double u)
{
	int span = findSpan(n, p, u, U);

	std::vector<double> N(p + 1);
	basisFuns(span, u, p, U, N);

	Point3d C;
	for (int i = 0; i <= p; i++){
		C = C + ctrl_pts[span - p + i] * N[i];
	}

	return C;
}


/**************************************************
@brief   : 得到曲线上某个点的向量
@author  : lee
@input   ：原曲线
		   p
		   U
		   ctrl_pts 
		   u 一般是从0到1的浮点数
@output  ：
@time    : none
**************************************************/
vector3d getDeriOfBSplineCurve(int p, std::vector<double> U, std::vector<Point3d> ctrl_pts, double u)
{
	int n = ctrl_pts.size() - 1;
	int pd = p - 1;
	int nd = n - 1;

	std::vector<double> Ud(U.size() - 2);
	copy(U.begin() + 1, U.end() - 1, Ud.begin());//如果要深度拷贝的化这个函数还是很方便的
	std::vector<Point3d> ctlptsD;
	for (int i = 0; i < n; i++)
		ctlptsD.push_back((ctrl_pts[i + 1] - ctrl_pts[i])*p*(U[i + p + 1] - U[i + 1]));
	Point3d deri = curvePoint(nd, pd, Ud, ctlptsD, u);
	vector3d deriv(deri.getX(), deri.getY(), deri.getZ());
	return deriv;
}


/**************************************************
@brief   : 根据书上公式计算求导后的曲线
@author  : lee
@input   ：原来样条曲线的 
		   p
		   U
		   ctrl_pts
@output  ：生成新的样条曲线
@time    : none
**************************************************/
BSplineCurveAbstract getDeriBSplineCurve(int p, std::vector<double> U, std::vector<Point3d> ctrl_pts) {
	int n = ctrl_pts.size() - 1;

	int pd = p - 1;// 因为求导导致的结果 p 阶次下降一
	int nd = n - 1;// m 下降两维度  n 也下降一个维度

	std::vector<double> Ud(U.size() - 2);
	copy(U.begin() + 1, U.end() - 1, Ud.begin());//生成新的U 前后都减少一个数
	std::vector<Point3d> ctlptsD;
	for (int i = 0; i < n; i++)
		ctlptsD.push_back((ctrl_pts[i + 1] - ctrl_pts[i])*p*(U[i + p + 1] - U[i + 1]));
	BSplineCurveAbstract curve(pd, nd, Ud, ctlptsD);
	return curve;
}


/* The Nurbs book page 151
Algorithm A5.1
Input:  np,p,UP,Pw,u,k,s,r
Output: nq,UQ,Qw
*/

void CurveKnotIns(int np, int p, std::vector<double> UP, std::vector<Point3d> Pw, double u, int s, int r, int &nq, std::vector<double> &UQ, std::vector<Point3d> &Qw)
{
	int mp = np + p + 1;
	nq = np + r;
	int k = findSpan(np, p, u, UP);  /* insert u span */

	std::vector<Point3d> Rw(p + 1);

	/*Load new knot vector*/
	for (int i = 0; i <= k; i++) UQ[i] = UP[i];
	for (int i = 1; i <= r; i++) UQ[k + i] = u;
	for (int i = k + 1; i <= mp; i++) UQ[i + r] = UP[i];

	/* Save unaltered control points*/
	for (int i = 0; i <= k - p; i++) Qw[i] = Pw[i];
	for (int i = k - s; i <= np; i++) Qw[i + r] = Pw[i];
	for (int i = 0; i <= p - s; i++) Rw[i] = Pw[k - p + i];

	/* Insert the knot r times*/
	int L;
	for (int j = 1; j <= r; j++)
	{
		L = k - p + j;
		for (int i = 0; i <= p - j - s; i++)
		{
			double alpha = (u - UP[L + i]) / (UP[i + k + 1] - UP[L + i]);
			Rw[i] = Rw[i + 1] * alpha + Rw[i] * (1 - alpha);
		}
		Qw[L] = Rw[0];
		Qw[k + r - j - s] = Rw[p - j - s];
	}

	/* Load remaining control points */
	for (int i = L + 1; i < k - s; i++)
		Qw[i] = Rw[i - L];
}

BSplineCurveAbstract curvePointKnontInsertion(int n, int p, std::vector<double> U, std::vector<Point3d> ctrl_pts, double insertu, int s, int insertTimes)
{
	int nq;
	std::vector<double>  UQ(U.size() + insertTimes);
	std::vector<Point3d> Qw(n + 1 + insertTimes);
	CurveKnotIns(n, p, U, ctrl_pts, insertu, s, insertTimes, nq, UQ, Qw);
	std::vector<Point3d> newControlPts = Qw;

	BSplineCurveAbstract curve(p, nq, UQ, Qw);

	return curve;
}