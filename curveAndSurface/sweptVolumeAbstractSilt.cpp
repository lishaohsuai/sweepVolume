#include "SweptVolumeAbstractSilt.h"
#include "cmath"


SweptVolumeAbstractSilt::SweptVolumeAbstractSilt(int p0_, int p1_, int p2_, std::vector<double> U0_, std::vector<double> U1_, std::vector<double> U2_, std::vector<std::vector<std::vector<Point3d>>> controlNet_)
{
	p0 = p0_;
	p1 = p1_;
	p2 = p2_;
	U0 = U0_;
	U1 = U1_;
	U2 = U2_;
	controlNet = controlNet_;
}

void SweptVolumeAbstractSilt::getTranslationalSweep(int p_, std::vector<double> knotU_, int q_, std::vector<double> knotV_, std::vector<Point3d> Unet, std::vector<Point3d> Vnet)
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
	//		controlNet[i][j] = Unet[i] + Vnet[j];// ���ɿ�������
}

/**************************************************
@brief   : ������ʾ
@author  : lee
@input   ��none
@output  ��p0 p1 ��ʾ��
		   p2 ��ʾ����
@time    : none
**************************************************/

void SweptVolumeAbstractSilt::outputSweptVolume(int &p0_, int &p1_, int &p2_, std::vector<double> &U0_, std::vector<double> &U1_, std::vector<double> &U2_, std::vector<std::vector<std::vector<Point3d>>> &controlNet_)
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
@brief   : ����ɨ��������ɨ����
			p0 �� p1 ��ʾ����
			p2 ��ʾ����  ����
@author  : lee
@input   ��T   B��������  �켣
		   Suv B��������
		   //Bv  ��ֱ��T ������  ��ʱû��ʹ�õ������������
		   Sx Sy   ���ŵ�B�������� // ��ʱ����
		   K  һ��ȡ10 Ҳ��֪����ʲô��
@output  ��controlNet  ���Ƶ�������Ŀ��Ƶ�
@time    : none
**************************************************/
void SweptVolumeAbstractSilt::getSweepVolumeSilt(BSplineCurveAbstract T, BSplineSurfaceAbstract Suv,  
	BSplineCurveAbstract Sx, BSplineCurveAbstract Sy, BSplineCurveAbstract Twist, int K) {
	p0 = Suv.getDegreeP();
	p1 = Suv.getDegreeQ();
	U0 = Suv.getKnotVecU();
	U1 = Suv.getKnotVecV();
	p2 = T.getDegree();
	int ktv = T.getKnotVec().size();
	int nsect = K + 1;

	if (ktv <= nsect + p2)//��ֵҲ�Ƕ�Ӧ��q��ֵ
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
	vSections[0] = 0;// ��С�Ĳ���ֵ
	vSections[nsect - 1] = 1;// ���Ĳ���ֵ
	for (int k = 1; k < nsect - 1; k++)
	{
		vSections[k] = 0;
		for (int j = 1; j <= p2; j++)
			vSections[k] += U2[k + j];
		vSections[k] = vSections[k] / p2;
	}

	// create transformed control points for each Section
	std::vector<std::vector<std::vector<Point3d>>> QTotalvFixed;  // vFixed for each row
	vector3d bi;
	vector3d Ti;
	vector3d Bi;
	for (int k = 0; k < nsect; k++)
	{
		double u = k * 1.0 * (T.getKnotVec()[T.getKnotVec().size() - 1] - T.getKnotVec()[0]) / nsect;//���ȷֳɶ��ٸ��������еĽڵ���
		std::vector<std::vector<Point3d>> Q = Suv.getControlPts();//std::vector<Point3d> Q = C.getControlPts();
		Matrix33 Sv = Matrix33::getIdentityMatrix();
		for (int i = 0; i < Q.size(); i++) {
			Point3d tmpx = Sx.getPointAt(u);
			Point3d tmpy = Sy.getPointAt(u);
			//std::cout << "[DEBUG] vSections[k]" << vSections[k] << std::endl;
			for (int j = 0; j < Q[i].size(); j++) {
				Sv.setElement(0, 0, tmpx.y);
				Sv.setElement(1, 1, tmpy.y);
				Sv.setElement(2, 2, 1);
				Q[i][j] = Sv * Q[i][j];
			}
		}  

		Matrix33 St = Matrix33::getIdentityMatrix();
		for (int i = 0; i < Q.size(); i++) {
			Point3d tmp = Twist.getPointAt(u);
			// ����ת�Ƕ�
			//double angle = tmp.y / 3.1415926 * 180.0;
			double angle = tmp.y;
			//std::cout << "[DEBUG] vSections[k]" << vSections[k] << std::endl;
			for (int j = 0; j < Q[i].size(); j++) {
				St.setElement(0, 0, cos(angle));
				St.setElement(0, 1, sin(angle) * -1);
				St.setElement(0, 2, 0);
				St.setElement(1, 0, sin(angle));
				St.setElement(1, 1, cos(angle));
				St.setElement(1, 2, 0);
				St.setElement(2, 2, 1);
				Q[i][j] = St * Q[i][j];
			}
		}

		// silt
		Ti = T.getFirstDerivative(u).normalize();
		if (k == 0) {
			vector3d B0;
			double delta = 0.001;
			while ((Ti.x == 0 && Ti.y == 0) ||
				(Ti.x == 0 && Ti.z == 0) ||
				(Ti.y == 0 && Ti.z == 0)) {
				Ti = T.getFirstDerivative(u + delta).normalize();
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
			//std::cout << " vertical2? " << Bi.x*Ti.x + Bi.y*Ti.y + Bi.z * Ti.z << std::endl;
			bi = Bi - Ti * (Bi.x*Ti.x + Bi.y*Ti.y + Bi.z*Ti.z);
			Bi = bi.normalize();
		}
		vector3d yv = T.getFirstDerivative(u).normalize();
		vector3d xv = Bi.normalize();//��silt������ Bvû�б�ʹ��
		vector3d zv = xv.crossp(yv);

		// for planar trajection curve, section curve facing y axis
		//vector3d yv = T.getFirstDerivative(vSections[k]).normalize();
		//vector3d xv = Bv.normalize();
		//vector3d zv = xv.crossp(yv);

		// for 3D trajectotion curve: use Frenet frame
		//vector3d td1 = T.getFirstDerivative(vSections[k]).normalize();
		//vector3d td2 = T.getDeriBSplineC().getFirstDerivative(vSections[k]).normalize();
		//vector3d yv = td1;// ��֪��ΪʲôҪ�� -1 ������ʵ�飿��  Bv û�������ﱻʹ��
		//vector3d xv = td1.crossp(td2); xv.normalize();
		//vector3d zv = xv.crossp(yv);

		Point3d pTv = T.getPointAt(u);
		//Matrix33 Av(xv, yv, zv);
		Matrix33 Av(zv, xv, yv);// ��ֱ��
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


void SweptVolumeAbstractSilt::insertKnots(BSplineCurveAbstract& T, int insertTimes)
{
	// insesrt on T curve
	for (int i = 0; i < insertTimes; i++)
	{
		double insertu = getMiddlePointOfLongestVSpan(T.getKnotVec());
		T = curvePointKnontInsertion(T.getn(), T.getDegree(), T.getKnotVec(), T.getControlPts(), insertu, 0, 1);
	}
}

double SweptVolumeAbstractSilt::getMiddlePointOfLongestVSpan(std::vector<double> KnotVec)
{
	int indLongSpan = 0;

	double MaxSpan = 0;
	for (int i = 0; i < KnotVec.size() - 1; i++)
	{
		double currentSpan = KnotVec[i + 1] - KnotVec[i];
		if (currentSpan > MaxSpan) {
			indLongSpan = i;
			MaxSpan = currentSpan;
		}
	}

	return (KnotVec[indLongSpan + 1] + KnotVec[indLongSpan]) / 2.0;
}