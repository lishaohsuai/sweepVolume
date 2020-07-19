#ifndef __VECTOR3D_H__
#define __VECTOR3D_H__
#include <math.h>
class vector3d {
public:
	double x, y, z;
	vector3d() { x = y = z = 0; }
	vector3d(double x_, double y_, double z_) :x(x_), y(y_), z(z_) {}
	vector3d(double p[3]) :x(p[0]), y(p[1]), z(p[2]) {}

	vector3d normalize() {
		double norm = sqrt(x*x + y * y + z * z);
		x = x / norm;
		y = y / norm;
		z = z / norm;
		return *this;
	}

	vector3d operator*(const double d) {
		return vector3d(x*d, y*d, z*d);
	}
	vector3d operator*(const vector3d d) {
		return vector3d(x * d.getx(), y * d.getx(), z * d.getz());
	}
	vector3d operator-(const vector3d d) {
		return vector3d(x - d.getx(), y - d.getx(), z - d.getz());
	}

	double getx() const { return x; }//����һ����Ա������ʱ����const�ؼ���������˵�����������
									 //"ֻ��(read-only)"������Ҳ����˵��������������޸��κ����ݳ�Ա(object)�� Ϊ������һ��const��Ա������ ��const�ؼ��ַ��ں������ŵĺ��档�����Ͷ����ʱ��Ӧ�÷�const�ؼ��֡�
	double gety() const { return y; }
	double getz() const { return z; }
	// ���������Ĳ��
	vector3d crossp(const vector3d &v) const {
		return vector3d(y*v.z - z * v.y,
			z*v.x - x * v.z,
			x*v.y - y * v.x);
	}
};



#endif
