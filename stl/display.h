#pragma once
#include <vector>
#include <stdio.h>
#include <tchar.h>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "glm/glm/glm.hpp"
#include "glm/glm/gtc/matrix_transform.hpp"
#include "glm/glm/gtc/type_ptr.hpp"

#include "cam_source.h"

using namespace std;
using cv::Mat;
using cv::Point;
using cv::Size;
using cv::Scalar;
using cv::waitKey;
using cv::WINDOW_NORMAL;
using cv::namedWindow;

void drawCADchar(vector<Point> contour, contourCharacteristics);
void drawCAMchar(contourCharacteristics, Mat);
void displayTmatrix(float(*matrix)[3]);
