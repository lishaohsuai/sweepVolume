#ifndef __STRATEGY_H__
#define __STRATEGY_H__
#include <iostream>
#include <json/config.h>
#include <json/json.h>
#include <QWidget.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#ifdef _DEBUG
#pragma comment(lib, "OpenMeshCored.lib")//这种链接库的方式似乎更好
#pragma comment(lib, "OpenMeshToolsd.lib")
#else
#pragma comment(lib, "OpenMeshCore.lib")
#pragma comment(lib, "OpenMeshTools.lib")
#endif
class Point3d {
public:
	double x;
	double y;
	double z;
public:
	Point3d(double _x, double _y, double _z) :x(_x), y(_y), z(_z) {};
	Point3d() { x = 0; y = 0; z = 0; };
	//getters
	double getX() const { return x; }
	double getY() const { return y; }
	double getZ() const { return z; }
	//operator
	Point3d operator*(double n) { return Point3d(x*n, y*n, z*n); }
	Point3d operator+(Point3d p) { return Point3d(x + p.x, y + p.y, z + p.z); }
	Point3d operator-(Point3d p) { return Point3d(x - p.x, y - p.y, z - p.z); }

	void scaleX(double s) { x = x / s; }
	void scaleY(double s) { y = y / s; }
	void scaleZ(double s) { z = z / s; }

	void print() { std::cout << "(" << x << "," << y << "," << z << ")" << std::endl; }
};
class Point2d {
public:
	double x;
	double y;
public:
	Point2d(double _x, double _y) :x(_x), y(_y) {};
	Point2d() { x = 0; y = 0;  };
	//getters
	double getX() const { return x; }
	double getY() const { return y; }
	//operator
	Point2d operator*(double n) { return Point2d(x*n, y*n); }
	Point2d operator+(Point2d p) { return Point2d(x + p.x, y + p.y); }
	Point2d operator-(Point2d p) { return Point2d(x - p.x, y - p.y); }

	void scaleX(double s) { x = x / s; }
	void scaleY(double s) { y = y / s; }

	void print() { std::cout << "(" << x << "," << y <<  ")" << std::endl; }
};
struct Point4d {
	double x;
	double y;
	double z;
	double w;// 权重
	Point4d(double _x = 0, double _y = 0, double _z = 0, double _w = 0) :x(_x), y(_y), z(_z), w(_w) {};
};
class Strategy{
public:
	Strategy();
	virtual ~Strategy();
	virtual void genPoints(std::string method);
	virtual void exportOBJ();

};


#endif