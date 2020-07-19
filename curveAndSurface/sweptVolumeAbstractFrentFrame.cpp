#include "SweptVolumeAbstractFrentFrame.h"


SweptVolumeAbstractFrentFrame::SweptVolumeAbstractFrentFrame(int p0_, int p1_, int p2_, std::vector<double> U0_, std::vector<double> U1_, std::vector<double> U2_, std::vector<std::vector<std::vector<Point3d>>> controlNet_)
{
	p0 = p0_;
	p1 = p1_;
	p2 = p2_;
	U0 = U0_;
	U1 = U1_;
	U2 = U2_;
	controlNet = controlNet_;
}

void SweptVolumeAbstractFrentFrame::getTranslationalSweep(int p_, std::vector<double> knotU_, int q_, std::vector<double> knotV_, std::vector<Point3d> Unet, std::vector<Point3d> Vnet)
{
	//p = p_;
	//q = q_;
	//knotU = knotU_;
	//knotV = knotV_;

	///*
	//   U: r = n + p + 1
	//   V: s = m + q + 1
	//*/
	//int r = knotU.size() - 1;
	//int s = knotV.size() - 1;
	//int n = r - p - 1;
	//int m = s - q - 1;
	//int N = n + 1; // number of control points
	//int M = m + 1; // number of control points

	//controlNet.resize(N);
	//for (int i = 0; i < N; i++)
	//	controlNet[i].resize(M);

	//for (int i = 0; i < N; i++)
	//	for (int j = 0; j < M; j++)
	//		controlNet[i][j] = Unet[i] + Vnet[j];// 生成控制网格
}

/**************************************************
@brief   : 输出体表示
@author  : lee
@input   ：none
@output  ：p0 p1 表示面
		   p2 表示长度
@time    : none
**************************************************/

void SweptVolumeAbstractFrentFrame::outputSweptVolume(int &p0_, int &p1_, int &p2_, std::vector<double> &U0_, std::vector<double> &U1_, std::vector<double> &U2_, std::vector<std::vector<std::vector<Point3d>>> &controlNet_)
{
	p0_ = p0;
	p1_ = p1;
	p2_ = p2;
	U0_.resize(U0.size());
	for (int i = 0; i < U0.size(); i++) {
		U0_[i] = U0[i];
	}
	U1_.resize(U1.size());
	for (int i = 0; i < U1.size(); i++) {
		U1_[i] = U1[i];
	}
	U2_.resize(U2.size());
	for (int i = 0; i < U2.size(); i++) {
		U2_[i] = U2[i];
	}
	controlNet_.resize(controlNet.size());
	for (int i = 0; i < controlNet.size(); i++) {
		controlNet_[i].resize(controlNet[i].size());
		for (int j = 0; j < controlNet[i].size(); j++) {
			controlNet_[i][j].resize(controlNet[i][j].size());
			for (int k = 0; k < controlNet[i][j].size(); k++) {
				controlNet_[i][j][k] = controlNet[i][j][k];
			}
		}
	}
}




/**************************************************
@brief   : 依据扫掠面生成扫掠体
			p0 和 p1 表示的面
			p2 表示的是  长度
@author  : lee
@input   ：T   B样条曲线  轨迹
		   Suv B样条曲面
		   Bv  垂直于T 的向量
		   S   缩放的B样条曲线 // 暂时不用
		   K  一般取10 也不知道有什么用
@output  ：controlNet  控制点样条体的控制点
@time    : none
**************************************************/
void SweptVolumeAbstractFrentFrame::getSweepSurface3(BSplineCurveAbstract T, BSplineSurfaceAbstract Suv, vector3d Bv, BSplineCurveAbstract S, int K) {
	p0 = Suv.getDegreeP();
	p1 = Suv.getDegreeQ();
	U0 = Suv.getKnotVecU();
	U1 = Suv.getKnotVecV();
	p2 = T.getDegree();
	int ktv = T.getKnotVec().size();
	int nsect = K + 1;

	if (ktv <= nsect + p2)//插值也是对应着q插值
	{ // must refine T(v)'s knot vector
		int m = nsect + p2 - ktv + 1;
		// insert kont in T for m times
		insertKnots(T, m);
		U2 = T.getKnotVec();
	}
	else {
		U2 = T.getKnotVec();
		if (ktv > nsect + p2 + 1) // must increase number of instance of C(u)
			nsect = ktv - p2 - 1;
	}

	// get section v-s
	std::vector<double> vSections(nsect);
	vSections[0] = 0;// 最小的参数值
	vSections[nsect - 1] = 1;// 最大的参数值
	for (int k = 1; k < nsect - 1; k++)
	{
		vSections[k] = 0;
		for (int j = 1; j <= p2; j++)
			vSections[k] += U2[k + j];
		vSections[k] = vSections[k] / p2;
	}

	// create transformed control points for each Section
	std::vector<std::vector<std::vector<Point3d>>> QTotalvFixed;  // vFixed for each row
	for (int k = 0; k < nsect; k++)
	{
		std::vector<std::vector<Point3d>> Q = Suv.getControlPts();//std::vector<Point3d> Q = C.getControlPts();
		Matrix33 Sv = Matrix33::getIdentityMatrix();
		//for (int i = 0; i < Q.size(); i++) {
		//	Point3d tmp = S.getPointAt(vSections[k]);
		//	std::cout << "[DEBUG] vSections[k]" << vSections[k] << std::endl;
		//	Sv.setElement(0, 0, tmp.getX());
		//	Sv.setElement(1, 1, tmp.getY());
		//	Sv.setElement(2, 2, tmp.getZ());
		//	for (int j = 0; j < Q[i].size(); j++) {
		//		Q[i][j] = Sv * Q[i][j];
		//	}// Q[i] = Sv * Q[i];
		//}  // 缩放暂时不实现把  有点乱
		//Q[i] = S * Q[i]; // scale every control point  Sv的值是随vSections[k]变化而变化的。吃完饭测试
		for (int i = 0; i < Q.size(); i++) {
			for (int j = 0; j < Q[i].size(); j++) {
				Q[i][j] = Sv * Q[i][j];
			}
		}



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
		//Matrix33 Av(xv, yv, zv);
		Matrix33 Av(zv, xv, yv);// 垂直的
		for (int i = 0; i < Q.size(); i++) {
			for (int j = 0; j < Q[i].size(); j++) {
				Q[i][j] = Av * Q[i][j] + pTv;
			}
		}
			//Q[i] = Av * Q[i] + pTv; // transform every control point

		//std::cout << "[DEBUG] QTotalvFixed " << k << std::endl;
		QTotalvFixed.push_back(Q);
	}

	// get the tranverse of QTotalvFixed
	int length = QTotalvFixed.size();
	int rows = QTotalvFixed[0].size();
	int cols = QTotalvFixed[0][0].size();
	std::vector<std::vector<std::vector<Point3d>>> QTotallengthFixed;
	QTotallengthFixed.resize(length);
	for (int i = 0; i < length; i++) {
		QTotallengthFixed[i].resize(rows);
		for (int j = 0; j < rows; j++) {
			QTotallengthFixed[i][j].resize(cols);
			for (int k = 0; k < cols; k++) {
				QTotallengthFixed[i][j][k] = QTotalvFixed[i][j][k];
			}
		}
	}
	//int n = C.getControlPts().size();
	//for (int i = 0; i < n; i++)
	{
		// TODO: interpolate across Q[0]....Q[nsect-1]
	}
	controlNet.resize(length);
	for (int i = 0; i < length; i++) {
		controlNet[i].resize(rows);
		for (int j = 0; j < rows; j++) {
			controlNet[i][j].resize(cols);
			for (int k = 0; k < cols; k++) {
				controlNet[i][j][k] = QTotallengthFixed[i][j][k];
			}
		}
	}
}

/**************************************************
@brief   : 依据扫掠面生成扫掠体
			p0 和 p1 表示的面
			p2 表示的是  长度
@author  : lee
@input   ：T   B样条曲线  轨迹
		   Suv B样条曲面
		   Bv  垂直于T 的向量
		   S   缩放的B样条曲线 // 暂时不用
		   K  一般取10 也不知道有什么用
@output  ：controlNet  控制点样条体的控制点
@time    : none
**************************************************/
void SweptVolumeAbstractFrentFrame::getSweepSurfacePlanar(BSplineCurveAbstract T, BSplineSurfaceAbstract Suv, vector3d Bv, BSplineCurveAbstract S, int K) {
	p0 = Suv.getDegreeP();
	p1 = Suv.getDegreeQ();
	U0 = Suv.getKnotVecU();
	U1 = Suv.getKnotVecV();
	p2 = T.getDegree();
	int ktv = T.getKnotVec().size();
	int nsect = K + 1;

	if (ktv <= nsect + p2)//插值也是对应着q插值
	{ // must refine T(v)'s knot vector
		int m = nsect + p2 - ktv + 1;
		// insert kont in T for m times
		insertKnots(T, m);
		U2 = T.getKnotVec();
	}
	else {
		U2 = T.getKnotVec();
		if (ktv > nsect + p2 + 1) // must increase number of instance of C(u)
			nsect = ktv - p2 - 1;
	}

	// get section v-s
	std::vector<double> vSections(nsect);
	vSections[0] = 0;// 最小的参数值
	vSections[nsect - 1] = 1;// 最大的参数值
	for (int k = 1; k < nsect - 1; k++)
	{
		vSections[k] = 0;
		for (int j = 1; j <= p2; j++)
			vSections[k] += U2[k + j];
		vSections[k] = vSections[k] / p2;
	}

	// create transformed control points for each Section
	std::vector<std::vector<std::vector<Point3d>>> QTotalvFixed;  // vFixed for each row
	for (int k = 0; k < nsect; k++)
	{
		std::vector<std::vector<Point3d>> Q = Suv.getControlPts();//std::vector<Point3d> Q = C.getControlPts();
		Matrix33 Sv = Matrix33::getIdentityMatrix();
		//for (int i = 0; i < Q.size(); i++) {
		//	Point3d tmp = S.getPointAt(vSections[k]);
		//	std::cout << "[DEBUG] vSections[k]" << vSections[k] << std::endl;
		//	Sv.setElement(0, 0, tmp.getX());
		//	Sv.setElement(1, 1, tmp.getY());
		//	Sv.setElement(2, 2, tmp.getZ());
		//	for (int j = 0; j < Q[i].size(); j++) {
		//		Q[i][j] = Sv * Q[i][j];
		//	}// Q[i] = Sv * Q[i];
		//}  // 缩放暂时不实现把  有点乱   缩放的效果捉摸不透
		//Q[i] = S * Q[i]; // scale every control point  Sv的值是随vSections[k]变化而变化的。吃完饭测试
		for (int i = 0; i < Q.size(); i++) {
			for (int j = 0; j < Q[i].size(); j++) {
				Q[i][j] = Sv * Q[i][j];
			}
		}



	// for planar trajection curve, section curve facing y axis
	vector3d yv = T.getFirstDerivative(vSections[k]).normalize();
	vector3d xv = Bv.normalize();
	vector3d zv = xv.crossp(yv);

	// for 3D trajectotion curve: use Frenet frame
		//vector3d td1 = T.getFirstDerivative(vSections[k]).normalize();
		//vector3d td2 = T.getDeriBSplineC().getFirstDerivative(vSections[k]).normalize();
		//vector3d yv = td1;// 不知道为什么要乘 -1 后面做实验？？  Bv 没有在这里被使用
		//vector3d xv = td1.crossp(td2); xv.normalize();
		//vector3d zv = xv.crossp(yv);

		Point3d pTv = T.getPointAt(vSections[k]);
		//Matrix33 Av(xv, yv, zv);//原来的  
		Matrix33 Av(zv, xv, yv);// 垂直的
		for (int i = 0; i < Q.size(); i++) {
			for (int j = 0; j < Q[i].size(); j++) {
				Q[i][j] = Av * Q[i][j] + pTv;
			}
		}
		//Q[i] = Av * Q[i] + pTv; // transform every control point

	//std::cout << "[DEBUG] QTotalvFixed " << k << std::endl;
		QTotalvFixed.push_back(Q);
	}

	// get the tranverse of QTotalvFixed
	int length = QTotalvFixed.size();
	int rows = QTotalvFixed[0].size();
	int cols = QTotalvFixed[0][0].size();
	std::vector<std::vector<std::vector<Point3d>>> QTotallengthFixed;
	QTotallengthFixed.resize(length);
	for (int i = 0; i < length; i++) {
		QTotallengthFixed[i].resize(rows);
		for (int j = 0; j < rows; j++) {
			QTotallengthFixed[i][j].resize(cols);
			for (int k = 0; k < cols; k++) {
				QTotallengthFixed[i][j][k] = QTotalvFixed[i][j][k];
			}
		}
	}
	//int n = C.getControlPts().size();
	//for (int i = 0; i < n; i++)
	{
		// TODO: interpolate across Q[0]....Q[nsect-1]
	}
	controlNet.resize(length);
	for (int i = 0; i < length; i++) {
		controlNet[i].resize(rows);
		for (int j = 0; j < rows; j++) {
			controlNet[i][j].resize(cols);
			for (int k = 0; k < cols; k++) {
				controlNet[i][j][k] = QTotallengthFixed[i][j][k];
			}
		}
	}
}

void SweptVolumeAbstractFrentFrame::insertKnots(BSplineCurveAbstract& T, int insertTimes)
{
	// insesrt on T curve
	for (int i = 0; i < insertTimes; i++)
	{
		double insertu = getMiddlePointOfLongestVSpan(T.getKnotVec());
		T = curvePointKnontInsertion(T.getn(), T.getDegree(), T.getKnotVec(), T.getControlPts(), insertu, 0, 1);
	}
}

double SweptVolumeAbstractFrentFrame::getMiddlePointOfLongestVSpan(std::vector<double> KnotVec)
{
	int indLongSpan = 0;

	double MaxSpan = 0;
	for (int i = 0; i < KnotVec.size() - 1; i++)
	{
		double currentSpan = KnotVec[i + 1] - KnotVec[i];
		if (currentSpan > MaxSpan){
			indLongSpan = i;
			MaxSpan = currentSpan;
		}
	}

	return (KnotVec[indLongSpan + 1] + KnotVec[indLongSpan]) / 2.0;
}