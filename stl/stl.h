#pragma once
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "glm/glm/glm.hpp"
#include "glm/glm/gtc/matrix_transform.hpp"
#include "glm/glm/gtc/type_ptr.hpp"


#define DEG_TO_RAD(x) (x*0.0174532925199f)

using namespace std;
using cv::Mat;
using cv::Point;
using cv::Size;
using cv::Scalar;
using cv::waitKey;
using cv::WINDOW_NORMAL;
using cv::namedWindow;

struct v3 {
	float x, y, z;
	v3(float _x = 0, float _y = 0, float _z = 0) : x(_x), y(_y), z(_z) {}
	float dotproduct(const v3 &v) const { return x*v.x + y*v.y + z*v.z; }
	void transform(const glm::mat4 &mat) { glm::vec4 v = glm::vec4(x, y, z, 1.0f), vt; vt = mat*v; x = vt.x; y = vt.y; z = vt.z; }
	v3& operator-=(const v3 &pt) { x -= pt.x; y -= pt.y; z -= pt.z; return *this; }
	v3 operator-(const v3 &pt) { return v3(x - pt.x, y - pt.y, z - pt.z); }
	v3 operator+(const v3 &pt) { return v3(x + pt.x, y + pt.y, z + pt.z); }
	v3 operator/(float a) { return v3(x / a, y / a, z / a); }
	v3 operator*(float a) { return v3(x*a, y*a, z*a); }
	float normalize() const { return sqrt(x*x + y*y + z*z); }
};

struct LineSegment {
	LineSegment(v3 p0 = v3(), v3 p1 = v3()) { v[0] = p0; v[1] = p1; }
	v3 v[2];
};

class Plane {
public:
	Plane() : mDistance(0) {}
	float distance() const { return mDistance; }
	float distanceToPoint(const v3 &vertex) const { return vertex.dotproduct(mNormal) - mDistance; }
	void setNormal(v3 normal) { mNormal = normal; }
	void setDistance(float distance) { mDistance = distance; }
protected:
	v3        mNormal;    // normalized Normal-Vector of the plane
	float   mDistance;  // shortest distance from plane to Origin
};

struct Triangle {
	Triangle(v3 n, v3 v0, v3 v1, v3 v2) : normal(n) { v[0] = v0; v[1] = v1; v[2] = v2; }
	Triangle& operator-=(const v3 &pt) { v[0] -= pt; v[1] -= pt; v[2] -= pt; return *this; }
	void transform(const glm::mat4 &mat) { v[0].transform(mat); v[1].transform(mat); v[2].transform(mat); }
	// @return -1 = all triangle is on plane back side
	//          0 = plane intersects the triangle
	//          1 = all triangle is on plane front side
	//         -2 = error in function
	int intersectPlane(const Plane &plane, LineSegment &ls) const; 
	v3 v[3], normal;
};

class TriangleMesh {
public:
	TriangleMesh() : bottomLeftVertex(999999, 999999, 999999), upperRightVertex(-999999, -999999, -999999) {}
	size_t size() const { return mesh.size(); }
	// move 3D Model coordinates to be center around COG(0,0,0)
	void normalize() {
		v3 halfBbox = (upperRightVertex - bottomLeftVertex) / 2.0f;
		v3 start = bottomLeftVertex + halfBbox;
		for (size_t i = 0; i<mesh.size(); ++i) {
			Triangle &triangle = mesh[i];
			triangle -= start;
		}
		bottomLeftVertex = halfBbox*-1.0f;
		upperRightVertex = halfBbox;
	}
	void push_back(const Triangle &t) {
		mesh.push_back(t);
		for (size_t i = 0; i<3; ++i) {
			if (t.v[i].x < bottomLeftVertex.x) bottomLeftVertex.x = t.v[i].x;
			if (t.v[i].y < bottomLeftVertex.y) bottomLeftVertex.y = t.v[i].y;
			if (t.v[i].z < bottomLeftVertex.z) bottomLeftVertex.z = t.v[i].z;
			if (t.v[i].x > upperRightVertex.x) upperRightVertex.x = t.v[i].x;
			if (t.v[i].y > upperRightVertex.y) upperRightVertex.y = t.v[i].y;
			if (t.v[i].z > upperRightVertex.z) upperRightVertex.z = t.v[i].z;
		}
	}
	v3 meshAABBSize() const {
		return v3(upperRightVertex.x - bottomLeftVertex.x, upperRightVertex.y - bottomLeftVertex.y, upperRightVertex.z - bottomLeftVertex.z);
	}
	std::vector<Triangle>& getMesh() { return mesh; }
	const std::vector<Triangle>& getMesh() const { return mesh; }
	v3 getBottomLeftVertex() const { return bottomLeftVertex; }
	v3 getUpperRightVertex() const { return upperRightVertex; }
	// Mesh COG point should be at (0,0,0)
	int transform(const glm::mat4 &mat) {
		for (size_t i = 0; i<mesh.size(); ++i) {
			Triangle &triangle = mesh[i];
			triangle.transform(mat);
		}
		return 0;
	}
	std::vector<Triangle> mesh;
	v3 bottomLeftVertex, upperRightVertex;
};

struct extrema {
	double max_y;
	double max_x;
	double min_y;
	double min_x;
};

vector<Point> getCADcontour();
vector<Point> formatOutputContour(vector<LineSegment> contour);
void displayContours(vector<std::vector<LineSegment>> slicesWithLineSegments, size_t nSlices);
int longestContour(std::vector<std::vector<LineSegment>>& slicesWithLineSegments, size_t nSlices);
double distance(float x1, float y1, float x2, float y2);
float slicesHeight(TriangleMesh &mesh, char orientation, int slicesAmount);
extrema find_scale(std::vector<std::vector<LineSegment>> slicesWithLineSegments, size_t nSlices);
int triMeshSlicer(const TriangleMesh *mesh, std::vector<std::vector<LineSegment>> &slicesWithLineSegments, const float sliceSize);
int stlToMeshInMemory(const char *stlFile, TriangleMesh *mesh);

