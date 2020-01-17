#include "transformMatricies.h"	

void setMaxima(contourCharacteristics& values, bool flag) {

	if (euclideanDist(values.center_of_mass, values.max_1) < euclideanDist(values.center_of_mass, values.max_2)) {
		int temp_x = values.max_1.x;
		int temp_y = values.max_1.y;
		values.max_1.x = values.max_2.x;
		values.max_1.y = values.max_2.y;
		values.max_2.x = temp_x;
		values.max_2.y = temp_y;
	}
}

void rotatePoint(Point &p, float angle) {
	float s = sin(angle);
	float c = cos(angle);

	int x = p.x * c - p.y * s;
	int y = p.x * s + p.y * c;
	p.x = x;
	p.y = y;
}

void callibrateCameraInput(contourCharacteristics &cam){		

	//callibration values
	float yReadCorrection, xReadCorrection, angleReadCorrection;
	std::ifstream infile("../data/calibrationCamera.txt");
	infile >> yReadCorrection;
	infile >> xReadCorrection;
	infile >> angleReadCorrection;
	//shift x and y values and flip Y axis
	cam.max_1.x -= xReadCorrection;
	cam.max_1.y = -(cam.max_1.y - yReadCorrection);
	cam.max_2.x -= xReadCorrection;
	cam.max_2.y = -(cam.max_2.y - yReadCorrection);
	cam.min_1.x -= xReadCorrection;
	cam.min_1.y = -(cam.max_1.y - yReadCorrection);
	cam.min_2.x -= xReadCorrection;
	cam.min_2.y = -(cam.max_2.y - yReadCorrection);
	cam.intersection.x -= xReadCorrection;
	cam.intersection.y = -(cam.intersection.y - yReadCorrection);
	cam.center_of_mass.x -= xReadCorrection;
	cam.center_of_mass.y = -(cam.center_of_mass.y - yReadCorrection);
	rotatePoint(cam.max_1, angleReadCorrection);
	rotatePoint(cam.max_2, angleReadCorrection);
	rotatePoint(cam.min_1, angleReadCorrection);
	rotatePoint(cam.min_2, angleReadCorrection);
	rotatePoint(cam.intersection, angleReadCorrection);
	rotatePoint(cam.center_of_mass, angleReadCorrection);

}

//Arguments: cad and cam fragment characteristics, return tanslation matrix and pass angle as reference
glm::mat4 findTmatrix(contourCharacteristics cam, float &angle) {

	//Switch values to make max1 closer to intersection point
	setMaxima(cam,1);

	callibrateCameraInput(cam);

	int y = cam.max_1.y - cam.max_2.y;
	int x = cam.max_1.x - cam.max_2.x;

	angle = atan2(y, x);

	glm::mat4 initTransMatrix = glm::mat4(1.0);
	initTransMatrix = initTransMatrix * glm::translate(vec3(cam.center_of_mass.x, cam.center_of_mass.y, 0.0f));

	return initTransMatrix;
}