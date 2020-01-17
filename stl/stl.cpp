#include "stl.h"

/*

	Mesh slicer written and shared by Raveh Gonen
	https://ravehgonen.wordpress.com/

*/



v3 operator-(const v3 &a, const v3 &b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
v3 operator+(const v3 &a, const v3 &b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }

int Triangle::intersectPlane(const Plane &plane, LineSegment &ls) const {
	// a triangle has 3 vertices that construct 3 line segments
	size_t cntFront = 0, cntBack = 0;
	for (size_t j = 0; j<3; ++j) {
		float distance = plane.distanceToPoint(v[j]);
		if (distance<0) ++cntBack;
		else ++cntFront;
	}
	if (3 == cntBack) {
		return -1;
	}
	else if (3 == cntFront) {
		return 1;
	}
	size_t lines[] = { 0,1,1,2,2,0 }; // CCW Triangle
	std::vector<v3> intersectPoints;
	for (size_t i = 0; i<3; ++i) {
		const v3 &a = v[lines[i * 2 + 0]];
		const v3 &b = v[lines[i * 2 + 1]];
		const float da = plane.distanceToPoint(a);
		const float db = plane.distanceToPoint(b);
		if (da*db<0) {
			const float s = da / (da - db); // intersection factor (between 0 and 1)
			v3 bMinusa = b - a;
			intersectPoints.push_back(a + bMinusa*s);
		}
		else if (0 == da) { // plane falls exactly on one of the three Triangle vertices
			if (intersectPoints.size()<2)
				intersectPoints.push_back(a);
		}
		else if (0 == db) { // plane falls exactly on one of the three Triangle vertices
			if (intersectPoints.size()<2)
				intersectPoints.push_back(b);
		}
	}
	if (2 == intersectPoints.size()) {
		// Output the intersecting line segment object
		ls.v[0] = intersectPoints[0];
		ls.v[1] = intersectPoints[1];
		return 0;
	}
	return -2;
}

// read the given STL file name ascii
// and generate a Triangle Mesh object in output parameter ‘mesh’
int stlToMeshInMemory(const char *stlFile, TriangleMesh *mesh) {
	ifstream in(stlFile);
	if (!in.good()) return 1;
	char title[80];
	std::string s0, s1;
	float n0, n1, n2, f0, f1, f2, f3, f4, f5, f6, f7, f8;
	in.read(title, 80);
	while (!in.eof()) {
		in >> s0;                                // facet || endsolid
		if (s0 == "facet") {
			in >> s1 >> n0 >> n1 >> n2;            // normal x y z
			in >> s0 >> s1;                        // outer loop
			in >> s0 >> f0 >> f1 >> f2;         // vertex x y z
			in >> s0 >> f3 >> f4 >> f5;         // vertex x y z
			in >> s0 >> f6 >> f7 >> f8;         // vertex x y z
			in >> s0;                            // endloop
			in >> s0;                            // endfacet
												 // Generate a new Triangle with Normal as 3 Vertices
			Triangle t(v3(n0, n1, n2), v3(f0, f1, f2), v3(f3, f4, f5), v3(f6, f7, f8));
			mesh->push_back(t);
		}
		else if (s0 == "endsolid") {
			break;
		}
	}
	in.close();
	//mesh->normalize();
	return 0;
}

// Take an input Triangle Mesh ‘mesh’ and fill the output
// parameter ‘slicesWithLineSegments’ with line segments for
// each slice
int triMeshSlicer(
	const TriangleMesh                        *mesh,        // the const input mesh
	std::vector<std::vector<LineSegment>>    &slicesWithLineSegments,
	// the result slices
	const float                                sliceSize) {// slice size in 3D Model digital units
	Plane plane;                                        // The intersection plane
	plane.setNormal(v3(0, 0, 1));                        // normal does not change during slicing
	const v3 aabb = mesh->meshAABBSize();                // as the model for it’s 3D axis-aligned bounding-box
	const size_t nSlices = 1 + (int)(aabb.z / sliceSize);    // compute number of output slices
	const std::vector<Triangle> &m = mesh->getMesh();    // get a const handle to the input mesh
	const float z0 = mesh->getBottomLeftVertex().z;        // find the minimal z coordinate of the model (z0)
	for (size_t i = 0; i<nSlices; ++i) {                    // start generating slices
		std::vector<LineSegment> linesegs;                // the linesegs vector for each slice
		plane.setDistance(z0 + (float)i*sliceSize);        // position the plane according to slice index
		for (size_t t = 0; t<m.size(); ++t) {                // iterate all mesh triangles
			const Triangle &triangle = m[t];            // get a const handle to a triangle
			LineSegment ls;
			if (0 == triangle.intersectPlane(plane, ls)) {// the plane does intersect the triangle
				linesegs.push_back(ls);                    // push a new Line Segment object to this slice
			}
		}
		slicesWithLineSegments.push_back(linesegs);        // push this vector to the slices vector
	}
	return 0;
}


extrema find_scale(std::vector<std::vector<LineSegment>> slicesWithLineSegments, size_t nSlices) {

	extrema values;
	const std::vector<LineSegment> &swls = slicesWithLineSegments[1];

	//set initial values
	(swls[0].v[0].x >= swls[0].v[1].x) ? values.max_x = swls[0].v[0].x : values.max_x = swls[0].v[1].x;
	(swls[0].v[0].y >= swls[0].v[1].y) ? values.max_y = swls[0].v[0].y : values.max_y = swls[0].v[1].y;
	(swls[0].v[0].x <= swls[0].v[1].x) ? values.min_x = swls[0].v[0].x : values.min_x = swls[0].v[1].x;
	(swls[0].v[0].y <= swls[0].v[1].y) ? values.min_y = swls[0].v[0].y : values.min_y = swls[0].v[1].y;

	//search maxs and mins in all slices
	for (int i = 1; i < (nSlices - 1); i++) {
		const std::vector<LineSegment> &lss = slicesWithLineSegments[i];
		//search in all lines
		for (size_t j = 0; j<lss.size(); ++j) {
			if (lss[j].v[0].x >= values.max_x)    values.max_x = (lss[j].v[0].x);
			if (lss[j].v[0].y >= values.max_y)    values.max_y = (lss[j].v[0].y);
			if (lss[j].v[0].x <= values.min_x)    values.min_x = (lss[j].v[0].x);
			if (lss[j].v[0].y <= values.min_y)    values.min_x = (lss[j].v[0].y);
			if (lss[j].v[1].x >= values.max_x)   values.max_x = (lss[j].v[1].x);
			if (lss[j].v[1].y >= values.max_y)   values.max_y = (lss[j].v[1].y);
			if (lss[j].v[1].x <= values.min_x)   values.min_x = (lss[j].v[1].x);
			if (lss[j].v[1].y <= values.min_y)   values.min_y = (lss[j].v[1].y);

		}
	}
	return values;
}

float slicesHeight(TriangleMesh &mesh, char orientation, int slicesAmount) {

	float max_height;
	float min_height;
	if (orientation == 'x') {
		max_height = mesh.mesh[0].v[0].x;
		min_height = mesh.mesh[0].v[0].x;
		for (int i = 0; i < mesh.mesh.size(); i++) {
			for (int j = 0; j < 3; j++) {
				if (mesh.mesh[i].v[j].x > max_height)
					max_height = mesh.mesh[i].v[j].x;
				if (mesh.mesh[i].v[j].z < min_height)
					min_height = mesh.mesh[i].v[j].z;
			}
		}
	}
	else if (orientation == 'y') {
		max_height = mesh.mesh[0].v[0].y;
		min_height = mesh.mesh[0].v[0].y;
		for (int i = 0; i < mesh.mesh.size(); i++) {
			for (int j = 0; j < 3; j++) {
				if (mesh.mesh[i].v[j].y > max_height)
					max_height = mesh.mesh[i].v[j].y;
				if (mesh.mesh[i].v[j].z < min_height)
					min_height = mesh.mesh[i].v[j].z;
			}
		}
	}
	else if (orientation == 'z') {
		max_height = mesh.mesh[0].v[0].z;
		min_height = mesh.mesh[0].v[0].z;
		for (int i = 0; i < mesh.mesh.size(); i++) {
			for (int j = 0; j < 3; j++) {
				if (mesh.mesh[i].v[j].z > max_height)
					max_height = mesh.mesh[i].v[j].z;
				if (mesh.mesh[i].v[j].z < min_height)
					min_height = mesh.mesh[i].v[j].z;
			}
		}
	}
	else
		return -1;

	float sliceSize = (max_height - min_height) / (float)slicesAmount;
	if (sliceSize > 0)
		return sliceSize;
	else
		return -1;
}

double distance(float x1, float y1, float x2, float y2) {
	float dx = x1 - x2;
	float dy = y1 - y2;
	return sqrt(dx*dx + dy*dy);
}


int longestContour(std::vector<std::vector<LineSegment>>& slicesWithLineSegments, size_t nSlices) {

	double maxSum = 0;
	int sliceIndex = -1;
	for (int i = 1; i < (nSlices - 1); i++) {
		const std::vector<LineSegment> &lss = slicesWithLineSegments[i];
		double sliceSum = 0;
		for (size_t j = 0; j<lss.size(); ++j) {
			sliceSum += distance(lss[j].v[0].x, lss[j].v[0].y, lss[j].v[1].x, lss[j].v[1].y);
		}
		if (sliceSum > maxSum) {
			maxSum = sliceSum;
			sliceIndex = i;
		}
	}

	if (sliceIndex >= 0)
		return sliceIndex;
	else
		return -1;
}

// Display slice after slice, press y+enter for next, press 0 to exit
void displayContours(vector<std::vector<LineSegment>> slicesWithLineSegments, size_t nSlices) {
	// Find max x,y coordinates of contour
	extrema values = find_scale(slicesWithLineSegments, nSlices);

	// Scale contour to displayed window
	int w_x = 400, w_y = 400;
	double scale = .01;

	double scale_x = (values.max_x*0.8) / (0.25*w_x);
	double scale_y = (values.max_y*0.8) / (0.25*w_y);
	bool stop = false;
	// Display segments
	for (int i = 1; i < (nSlices - 1); i++) {
		char layer1_window[] = "Layer 1";
		Mat layer1_image = Mat::zeros(w_x, w_x, CV_8UC3);

		const std::vector<LineSegment> &lss = slicesWithLineSegments[i];
		for (size_t j = 0; j < lss.size(); j++)
		{
			circle(layer1_image, Point((w_x / 2) + lss[j].v[0].x / scale_x, (w_y / 2) + lss[j].v[0].y / scale_y), 4, Scalar(250, 0, 0), 1, 8, 0);
			circle(layer1_image, Point((w_x / 2) + lss[j].v[1].x / scale_x, (w_y / 2) + lss[j].v[1].y / scale_y), 4, Scalar(250, 0, 0), 1, 8, 0);
			//line(layer1_image, Point((w_x / 2) + lss[j].v[0].x / scale_x, (w_y / 2) + lss[j].v[0].y / scale_y), Point((w_x / 2) + lss[j].v[1].x / scale_x, (w_y / 2) + lss[j].v[1].y / scale_y), Scalar(250, 0, 0), 2, 8);
		}
		imshow(layer1_window, layer1_image);
		// Wait for y+enter press to continue displaying contours
		printf("\n\nPress y+enter to show next contour\n");
		int press_button = 0;
		do {
			imshow(layer1_window, layer1_image);
			waitKey(1);
			press_button = getchar();
			if (press_button == 48)
				stop = true;
		} while (press_button != 121);
		if (stop == true)
			break;
	}
}

vector<Point> formatOutputContour(vector<LineSegment> contour) {
	vector<Point> output_contour;
	Point temp;
	for (int i = 0; i < contour.size(); i++) {
		for (int j = 0; j < 2; j++) {
			temp.x = (int) 100 * contour[i].v[j].x;
			temp.y = (int) 100 * contour[i].v[j].y;
			output_contour.push_back(temp);
		}
	}

	return output_contour;
}



vector<Point> getCADcontour() {
	char    modelFileName[1024] = "../data/model_shoe/shoeModel.txt";
	glm::tvec3<float> eulerAngles(0, 0, 0);


	// Parse the file and generate a TriangleMesh
	TriangleMesh mesh;
	if (stlToMeshInMemory(modelFileName, &mesh) != 0)
		printf("Nie uda³o siê odczytaæ pliku CAD");

	// Based on mesh height and slices amount find slices height  
	int slicesAmount = 40;
	float sliceSize = slicesHeight(mesh, 'z', slicesAmount);
	if (sliceSize == -1)
		printf("Nie uda³o siê uzyskaæ segmentów");

	// Optional Model Rotation Around COG Point using GLM Library
	glm::mat4 mat = glm::mat4(1.0f);
	glm::tquat<float> quaternion = glm::tquat<float>(DEG_TO_RAD(eulerAngles)); // This glm func wants values as radians
	float angle = glm::angle(quaternion);
	glm::tvec3<float> axis = glm::axis(quaternion);
	mat = glm::rotate(mat, angle, axis);
	mesh.transform(mat);

	// Generate Slices
	std::vector<std::vector<LineSegment>> slicesWithLineSegments;
	triMeshSlicer(&mesh, slicesWithLineSegments, sliceSize);

	const size_t nSlices = slicesWithLineSegments.size();
	const size_t slicePerRow = (size_t)sqrt((float)nSlices);
	const v3 &aabbSize = mesh.meshAABBSize();

	// OPTIONAL: Display slice after slice, press y+enter for next
	//displayContours(slicesWithLineSegments, nSlices);

	// Find the longest contour
	//int indexMaxContour = longestContour(slicesWithLineSegments, nSlices);
	int indexMaxContour = 9;
	vector<Point> outputSegment = formatOutputContour(slicesWithLineSegments[indexMaxContour]);
	return outputSegment;
}
