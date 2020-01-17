#include "stdafx.h"
#include <vector>
#include <stdio.h>
#include <tchar.h>
#include <math.h>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "glm/glm/glm.hpp"
#include "glm/glm/gtc/matrix_transform.hpp"
#include "glm/glm/gtc/type_ptr.hpp"

#include "stl.h"
#include "cam_source.h"
#include "display.h"
#include "insGPU.h"
#include "transformMatricies.h"	

using namespace std;
using cv::Mat;
using cv::Point;
using cv::Size;
using cv::Scalar;
using cv::waitKey;
using cv::WINDOW_NORMAL;
using cv::namedWindow;


int main(int argc, char **argv) {
	
	// Set camera to the center of the device
	int projectorCalibration = 0, cameraCalibration = 0;
	
	// Manually adjust position of the  projector 
	// such that the projected dot from the projector 
	// is in the middle of the table, directly above it
	contourCharacteristics cam_char;
	cam_char = camCapture(cameraCalibration);
	Mat cam_src = getCAMsrc();
	drawCAMchar(cam_char, cam_src);
	waitKey(0);
	float angle;
	// Find the translation matrix obtained from the above calibration to adjust 
	// positions between the projector and the table
	glm::mat4 transInitMatrix = findTmatrix(cam_char, angle);

	startProjection(projectorCalibration, angle, transInitMatrix);

	return 0;
}

