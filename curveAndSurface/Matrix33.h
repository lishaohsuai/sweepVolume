#ifndef __MATRIX33_H__
#define __MATRIX33_H__

class vector3d;
class Point3d;

class Matrix33 {
private:
	double m[3][3];
public:
	Matrix33() {}
	Matrix33(double m_[3][3]);
	Matrix33(vector3d v1, vector3d v2, vector3d v3);

	Matrix33 operator-(const Matrix33& other) const;
	Matrix33 operator*(double d) const;
	vector3d operator*(const vector3d& other) const;
	Point3d  operator*(const Point3d& other) const;

	Matrix33 getInverse()const;
	void   setElement(int i, int j, double v);
	double getElement(int i, int j);
	Matrix33 getTranspose()const;
	void print();

	static Matrix33 getIdentityMatrix();
};


#endif