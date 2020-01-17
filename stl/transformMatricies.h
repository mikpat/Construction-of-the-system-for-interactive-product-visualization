#pragma once
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
#include <glm/glm/gtx/transform.hpp>
using namespace glm;

#include "stl.h"
#include "cam_source.h"
#include "cam_source.cpp""


glm::mat4 findTmatrix(contourCharacteristics, float &);
void setMaxima(contourCharacteristics&, bool);
void callibrateCameraInput(contourCharacteristics &);

