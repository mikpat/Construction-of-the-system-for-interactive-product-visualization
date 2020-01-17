#pragma once

// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <windows.h>
#include <fstream>
#include <math.h>
#include <sstream>
#include <thread>
#include <future>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <glfw3.h>
GLFWwindow* window;

// Include GLM
#include "glm/glm/glm.hpp"
#include "glm/glm/gtc/matrix_transform.hpp"
#include <glm/glm/gtc/quaternion.hpp>
#include <glm/glm/gtx/quaternion.hpp>
#include <glm/glm/gtx/transform.hpp>
#include <glm/glm/gtx/matrix_decompose.hpp>
#include <glm/glm/gtx/rotate_vector.hpp>
#include <glm/glm/gtc/type_ptr.hpp> 

#include <common/shader.hpp>
#include <common/texture.hpp>
#include <common/controls.hpp>
#include <common/objloader.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using cv::Mat;
using namespace glm;
using namespace std;

int startProjection(int, float, glm::mat4);
cv::Point markerPosition(cv::VideoCapture &);
void markerPosition2(cv::VideoCapture &, cv::Point *p);
float angleRotation(cv::Point, cv::Point, std::ofstream &);
int distance(cv::Point& p, cv::Point& q);
Mat findMarker(Mat &scr_image);