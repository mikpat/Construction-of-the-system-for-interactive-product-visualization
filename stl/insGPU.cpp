#include "insGPU.h"

/*

	Code based on series of tutorials from:
	http://www.opengl-tutorial.org

*/

int outputIndex = 0;
int threshMarker = 160;
float shiftScene = 30;

int startProjection(int projectorCalibration, float init_rotation_angle_radians, glm::mat4 init_trans_matrix)
{
	float angleMarker = 0;
	cv::VideoCapture cap(0); // open the default camera
	cap.set(CV_CAP_PROP_BUFFERSIZE, 1);
	if (!cap.isOpened()) {
		cout << "\n\nNie udalo sie otworzyc kamery do pobrania obrazu markera.\n\n";
		return -1;
	}
	init_trans_matrix[3].x = 0.13*init_trans_matrix[3].x;
	init_trans_matrix[3].z = -(100 / 480.0)*init_trans_matrix[3].y;
	init_trans_matrix[3].y = 0;
	float scaleValue;
	glm::mat4 transModelMatrix;
	glm::mat4 rotateModelMatrix = glm::mat4(1.0);
	if (projectorCalibration == 1) {
		scaleValue = 0.166;
		transModelMatrix = init_trans_matrix;
	}
	else if (projectorCalibration == 2) {
		scaleValue = 1;
		init_trans_matrix = glm::mat4(1.0);
		init_rotation_angle_radians = 0;
		transModelMatrix = init_trans_matrix;
	}
	else {
		float yReadCorrection, xReadCorrection, scaleReadCorrection, angleReadCorrection;
		std::ifstream infile("../data/calibrationProjector.txt");
		infile >> xReadCorrection;
		infile >> yReadCorrection;
		infile >> scaleReadCorrection;
		infile >> angleReadCorrection;
		init_rotation_angle_radians += angleReadCorrection;
		transModelMatrix = init_trans_matrix;
		transModelMatrix *= glm::translate(vec3(xReadCorrection, 0.0f, yReadCorrection));
		rotateModelMatrix = glm::mat4(1.0);
		scaleValue = scaleReadCorrection;
	}
	glm::vec3 scaleModel = vec3(scaleValue);
	glm::vec3 initDistance = glm::vec3(transModelMatrix[3].x, 0, transModelMatrix[3].z);
	const float rotationRadius = glm::length(initDistance);

	rotateModelMatrix = glm::rotate(rotateModelMatrix, init_rotation_angle_radians, glm::vec3(0, 1, 0));


	// Initialise GLFW
	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		getchar();
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);


	//Find all monitor, primary monitor is first one, second 1024x768
	int count;
	GLFWmonitor** monitors = glfwGetMonitors(&count);
	window = glfwCreateWindow(1600, 900, "Main window", *(monitors + count - 2), NULL);

	// Open a window and create its OpenGL context
	if (window == NULL) {
		fprintf(stderr, "Failed to open GLFW window.\n");
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
	// Hide the mouse and enable unlimited mouvement
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// Set the mouse at the center of the screen
	glfwPollEvents();
	glfwSetCursorPos(window, 1024 / 2, 768 / 2);

	// Background Color
	if (projectorCalibration == 2)
		glClearColor(255.0f, 255.0f, 255.0f, 254.0f);
	else
		glClearColor(0.0f, 0.0f, 0.0f, 254.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

	// Create and compile our GLSL program from the shaders
	GLuint programID = LoadShaders("TransformVertexShader.vertexshader", "TextureFragmentShader.fragmentshader");

	// Get a handle for our "MVP" uniform
	GLuint MatrixID = glGetUniformLocation(programID, "MVP");

	// Get a handle for our buffers
	GLuint vertexPosition_modelspaceID = glGetAttribLocation(programID, "vertexPosition_modelspace");
	GLuint vertexUVID = glGetAttribLocation(programID, "vertexUV");

	// Load the texture
	GLuint Texture;
	if (projectorCalibration == 2)
		Texture = loadDDS("../data/model_calibration/uvCalibration.DDS");
	else
		Texture = loadDDS("../data/model_shoe/shoeUV.DDS");
	// Get a handle for our "myTextureSampler" uniform
	GLuint TextureID = glGetUniformLocation(programID, "myTextureSampler");

	// Read our .obj file
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> uvs;
	std::vector<glm::vec3> normals; // Won't be used at the moment.
	if (projectorCalibration == 2)
		loadOBJ("../data/model_calibration/Calibration.obj", vertices, uvs, normals);
	else
		loadOBJ("../data/model_shoe/shoeModel.obj", vertices, uvs, normals);


	// Load it into a VBO
	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

	GLuint uvbuffer;
	glGenBuffers(1, &uvbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), &uvs[0], GL_STATIC_DRAW);

	cv::Point previousMarker;
	if (projectorCalibration == 0) {
		do {
			previousMarker = markerPosition(cap);
		} while (previousMarker == cv::Point(0, 0));
	}
	//Values for counting correction 
	float leftCorrection = 0, rightCorrection = 0, upCorrection = 0, downCorrection = 0, scaleCorrection = 0, rotationCorrection=0;
	//Resolution
	float singleDisplacement = 0.02;

	glm::vec3 previoustranslationModel;
	glm::mat4 ModelMatrix;

	cv::Point currentMarker;
	std::ofstream markerFile("../data/marker.txt", std::ios_base::app | std::ios_base::out);
	int countmarketimes = 0;
	bool firstrotate = 1;

	do {

		if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) {
			Texture = loadDDS("../data/model_shoe/shoeUV.DDS");
			TextureID = glGetUniformLocation(programID, "myTextureSampler");
		}
		if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS) {
			Texture = loadDDS("../data/model_shoe/shoeUV_2.DDS");
			TextureID = glGetUniformLocation(programID, "myTextureSampler");
		}
		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Use our shader
		glUseProgram(programID);

		// Compute the MVP matrix
		computeMatricesFromInputs();
		glm::mat4 ProjectionMatrix = getProjectionMatrix();
		glm::mat4 ViewMatrix = getViewMatrix();

		if (projectorCalibration != 1) {
			currentMarker = markerPosition(cap);
			//If didn't find marker
			if (currentMarker == cv::Point(0, 0) || distance(currentMarker, previousMarker) > 400) {
				angleMarker = 0;
			}
			else {
				angleMarker = angleRotation(previousMarker, currentMarker, markerFile);
				previousMarker = currentMarker;
			}
		}
		markerFile << angleMarker << endl;

		//Rotation in clockwise direction
		//rotation_angle_deg = rotation_angle_radians*(3.1415/180);
		float rotation_angle_radians = -0.00349 * 15;
		float rotation_angle_radians2 = 0.00349 * 15;
		float rotation_while_callibration = 0.00349 * 4;

		glm::mat4 transformationModel = ModelMatrix;
		glm::vec3 scaleM;
		glm::quat rotationModel;
		glm::vec3 translationModel;
		glm::vec3 skewModel;
		glm::vec4 perspectiveModel;
		glm::decompose(transformationModel, scaleM, rotationModel, translationModel, skewModel, perspectiveModel);

		translationModel = glm::normalize(translationModel);
		translationModel = rotationRadius * translationModel;

		if (projectorCalibration == 1) {
			if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
				transModelMatrix = transModelMatrix * glm::translate(vec3(singleDisplacement, 0.0f, 0.0f));
				rightCorrection++;
			}
			if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
				transModelMatrix = transModelMatrix * glm::translate(vec3((-1 * singleDisplacement), 0.0f, 0.0f));
				leftCorrection++;
			}
			if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
				transModelMatrix = transModelMatrix * glm::translate(vec3(0.0f, 0.0f, (-1 * singleDisplacement)));
				upCorrection++;
			}
			if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
				transModelMatrix = transModelMatrix * glm::translate(vec3(0.0f, 0.0f, singleDisplacement));
				downCorrection++;
			}
			if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS) {
				scaleValue += 0.0004;
				scaleModel = vec3(scaleValue);
			}
			if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS) {
				scaleValue -= 0.0004;
				scaleModel = vec3(scaleValue);
				}
			if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
				rotationCorrection += 0.001;
				rotateModelMatrix = glm::rotate(rotateModelMatrix, 0.002f, glm::vec3(0, 1, 0));
			}
			if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) {
				rotationCorrection -= 0.001;
				rotateModelMatrix = glm::rotate(rotateModelMatrix, -0.002f, glm::vec3(0, 1, 0));
			}
		}
		else {
				if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
					rotateModelMatrix = glm::rotate(rotateModelMatrix, rotation_angle_radians, glm::vec3(0, 1, 0));

					translationModel = glm::vec3(-1.0f, 0, -1.0f) * translationModel;
					transModelMatrix = glm::translate(transModelMatrix, translationModel);
					translationModel = glm::vec3(-1.0f, 0, -1.0f) * translationModel;
					translationModel = glm::rotate(translationModel, rotation_angle_radians, glm::vec3(0, 1, 0));
					transModelMatrix = glm::translate(transModelMatrix, translationModel);

					if (translationModel != previoustranslationModel)
						printf("Len: %.3f\n", glm::length(translationModel));
					previoustranslationModel = translationModel;

				}
				if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
					rotateModelMatrix = glm::rotate(rotateModelMatrix, rotation_angle_radians2, glm::vec3(0, 1, 0));

					translationModel = glm::vec3(-1.0f, 0, -1.0f) * translationModel;
					transModelMatrix = glm::translate(transModelMatrix, translationModel);
					translationModel = glm::vec3(-1.0f, 0, -1.0f) * translationModel;
					translationModel = glm::rotate(translationModel, rotation_angle_radians2, glm::vec3(0, 1, 0));
					transModelMatrix = glm::translate(transModelMatrix, translationModel);

					if (translationModel != previoustranslationModel)
						printf("Len: %.3f\n", glm::length(translationModel));
					previoustranslationModel = translationModel;
				}
				if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
					rotationCorrection += 0.008;
					rotateModelMatrix = glm::rotate(rotateModelMatrix, 0.008f, glm::vec3(0, 1, 0));
				}
				if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) {
					rotationCorrection -= 0.008;
					rotateModelMatrix = glm::rotate(rotateModelMatrix, -0.008f, glm::vec3(0, 1, 0));
				}
				if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
					scaleValue += 0.04;
					scaleModel = vec3(scaleValue);
				}
				if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
					scaleValue -= 0.04;
					scaleModel = vec3(scaleValue);
				}
				if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
					transModelMatrix = transModelMatrix * glm::translate(vec3(singleDisplacement, 0.0f, 0.0f));
					rightCorrection++;
				}
				if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
					transModelMatrix = transModelMatrix * glm::translate(vec3((-1 * singleDisplacement), 0.0f, 0.0f));
					leftCorrection++;
				}
				if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
					transModelMatrix = transModelMatrix * glm::translate(vec3(0.0f, 0.0f, (-1 * singleDisplacement)));
					upCorrection++;
				}
				if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
					transModelMatrix = transModelMatrix * glm::translate(vec3(0.0f, 0.0f, singleDisplacement));
					downCorrection++;
				}
			}
			if (projectorCalibration == 0) {
				if (firstrotate != 1) {
					rotateModelMatrix = glm::rotate(rotateModelMatrix, angleMarker, glm::vec3(0, 1, 0));

					translationModel = glm::vec3(-1.0f, 0, -1.0f) * translationModel;
					transModelMatrix = glm::translate(transModelMatrix, translationModel);
					translationModel = glm::vec3(-1.0f, 0, -1.0f) * translationModel;
					translationModel = glm::rotate(translationModel, angleMarker, glm::vec3(0, 1, 0));
					transModelMatrix = glm::translate(transModelMatrix, translationModel);
					previoustranslationModel = translationModel;
				}
				firstrotate = 0;
				countmarketimes++;
			}

			ModelMatrix = glm::mat4(1.0);
			ModelMatrix = glm::scale(ModelMatrix, scaleModel);
			ModelMatrix = transModelMatrix * rotateModelMatrix * ModelMatrix;
			glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

			// Send our transformation to the currently bound shader, 
			// in the "MVP" uniform
			glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

			// Bind our texture in Texture Unit 0
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, Texture);
			// Set our "myTextureSampler" sampler to user Texture Unit 0
			glUniform1i(TextureID, 0);

			// 1rst attribute buffer : vertices
			glEnableVertexAttribArray(vertexPosition_modelspaceID);
			glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
			glVertexAttribPointer(
				vertexPosition_modelspaceID,  // The attribute we want to configure
				3,                            // size
				GL_FLOAT,                     // type
				GL_FALSE,                     // normalized?
				0,                            // stride
				(void*)0                      // array buffer offset
			);

			// 2nd attribute buffer : UVs
			glEnableVertexAttribArray(vertexUVID);
			glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
			glVertexAttribPointer(
				vertexUVID,                   // The attribute we want to configure
				2,                            // size : U+V => 2
				GL_FLOAT,                     // type
				GL_FALSE,                     // normalized?
				0,                            // stride
				(void*)0                      // array buffer offset
			);

			// Draw the triangles !
			glDrawArrays(GL_TRIANGLES, 0, vertices.size());

			glDisableVertexAttribArray(vertexPosition_modelspaceID);
			glDisableVertexAttribArray(vertexUVID);

			// Swap buffers
			glfwSwapBuffers(window);
			glfwPollEvents();

		} // Check if the ESC key was pressed or the window was closed
		while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
			glfwWindowShouldClose(window) == 0);


		printf("\n\n%d", countmarketimes);
		// Cleanup VBO and shader
		glDeleteBuffers(1, &vertexbuffer);
		glDeleteBuffers(1, &uvbuffer);
		glDeleteProgram(programID);
		glDeleteTextures(1, &TextureID);

		// Close OpenGL window and terminate GLFW
		glfwTerminate();


		if (projectorCalibration == 1) {
			ofstream report("../data/calibrationProjector.txt");
			if (report.is_open())
			{
				report << (rightCorrection*singleDisplacement - leftCorrection*singleDisplacement) << endl;
				report << (downCorrection*singleDisplacement - upCorrection*singleDisplacement) << endl;
				report << scaleValue << endl;
				report << rotationCorrection << endl;
				report << "Calibration projector values report\n\n";
				report << "X Y Scale" << endl;
			}
			else
				cout << "\n\nUnable to open callibration report\n\n";
		}
		return 0;
}


cv::Point markerPosition(cv::VideoCapture &cap) {
	Mat imageCam, imageCam_gray, threshImage, threshImage2, morph;

	//Mat cam_init_projection;
	//cam_init_projection = cv::imread("../data/projection_black.png", CV_LOAD_IMAGE_COLOR);
	//cvNamedWindow("CamInit", CV_WINDOW_NORMAL);
	//cvSetWindowProperty("CamInit", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	//imshow("CamInit", cam_init_projection);
	//cv::waitKey(10);

	cap >> imageCam;

	if (!imageCam.data){
		cout << "\nNie udalo sie pobrac obrazu do znalezienia markera\n";
		return 0;
	}
	imageCam = imageCam(cv::Rect(120, 0, 520, 480));
	vector<Mat> hsvChannels(3);
	cvtColor(imageCam, imageCam_gray, cv::COLOR_BGR2HSV);
	split(imageCam_gray, hsvChannels);
	inRange(hsvChannels[2], threshMarker, 255, threshImage);

	dilate(threshImage, threshImage, Mat(), cv::Point(-1, -1), 2);
	//erode(threshImage, threshImage, Mat(), cv::Point(-1,-1), 3);
	//morphologyEx(threshImage, morph, cv::MORPH_CLOSE, Mat(), cv::Point(-1, -1), 3);
	cv::threshold(threshImage, threshImage2, 0.0, 1.0, cv::THRESH_BINARY);
	//namedWindow("prog", WINDOW_NORMAL);
	//imshow("prog", morphologySrc);
	//cv::waitKey(0);
	Mat objectDetection = findMarker(threshImage2);

	GaussianBlur(objectDetection, objectDetection, cv::Size(9, 9), 2, 2);
	//cv::namedWindow("Hough Circle", CV_WINDOW_AUTOSIZE);
	//cv::imshow("Hough Circle", objectDetection);
	//cv::waitKey(0);

	vector<cv::Vec3f> circles;
	HoughCircles(objectDetection, circles, CV_HOUGH_GRADIENT, 1, objectDetection.rows / 2, 200, 10, 14, 18);

	//for (size_t i = 0; i < circles.size(); i++)
	//{
	//	cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	//	int radius = cvRound(circles[i][2]);
	//	circle(objectDetection, center, radius, cv::Scalar(0, 255, 0), 3, 8, 0);
	//}

	if (circles.size() != 1) {
		//std::ostringstream name;
		//name << "../data/images/failed_" << outputIndex << ".png";
		//cv::imwrite(name.str(), objectDetection);
		//outputIndex++;
		//std::ostringstream name2;
		//name2 << "../data/images/failedThresh_" << outputIndex << ".png";
		//cv::imwrite(name2.str(), threshImage);
		return cv::Point(0, 0);
	}


	//std::ostringstream name;
	//name << "../data/images/rotated_im_" << outputIndex << ".png";
	//cv::imwrite(name.str(), objectDetection);
	//std::ostringstream name2;
	//name2 << "../data/images/rotated_im_good" << outputIndex << ".png";
	//cv::imwrite(name2.str(), threshImage);
	//outputIndex++;


	return cv::Point(circles[0][0], circles[0][1]);
}

//Return angle radians change between two points
float angleRotation(cv::Point previousPoint, cv::Point currentPoint, std::ofstream &markerFile){

	int yReadCorrection, xReadCorrection;
	std::ifstream infile("../data/calibrationCamera.txt");
	infile >> yReadCorrection;
	infile >> xReadCorrection;
	float x1 = previousPoint.x- xReadCorrection;
	float y1 = -(previousPoint.y- yReadCorrection);

	float x2 = currentPoint.x- xReadCorrection;
	float y2 = -(currentPoint.y- yReadCorrection);

	//markerFile << "x=" << x2 << " y=" << y2 << "  ";

	float dot = x1*x2 + y1*y2;
	float det = x1*y2 - y1*x2;
	// In deg: angle = atan2(det, dot) * 180 / 3.14;
	if (dot == 0)
		return 0;
	float angle = atan2(det, dot);

	return angle;
}

int distance(cv::Point& p, cv::Point& q) {
	cv::Point d = p - q;
	return (int)sqrt(d.x*d.x + d.y*d.y);
}

Mat findMarker(Mat &scr_image) {

	std::vector < std::vector<cv::Point2i> > blobs;
	cv::Mat label_image;
	scr_image.convertTo(label_image, CV_32SC1);

	int label_count = 2; // starts at 2 because 0,1 are used already

	for (int y = 0; y < label_image.rows; y++) {
		int *row = (int*)label_image.ptr(y);
		for (int x = 0; x < label_image.cols; x++) {
			if (row[x] != 1) {
				continue;
			}

			cv::Rect rect;
			cv::floodFill(label_image, cv::Point(x, y), label_count, &rect, 0, 0, 4);

			std::vector <cv::Point2i> blob;

			for (int i = rect.y; i < (rect.y + rect.height); i++) {
				int *row2 = (int*)label_image.ptr(i);
				for (int j = rect.x; j < (rect.x + rect.width); j++) {
					if (row2[j] != label_count) {
						continue;
					}

					blob.push_back(cv::Point2i(j, i));
				}
			}

			blobs.push_back(blob);

			label_count++;
		}
	}

	vector<int> marker_index;
	for (int i = 0; i < blobs.size(); i++) {
		vector<cv::Point> obj = blobs[i];
		if(600 < static_cast<int>(obj.size()) && (static_cast<int>(obj.size()) < 1300)) {
			marker_index.push_back(i);
		}
	}
	Mat output = Mat::zeros(scr_image.size(), CV_8UC1);
	Mat output2 = Mat::zeros(scr_image.size(), CV_8UC1);;
	for(size_t j=0; j < marker_index.size(); j++) {
		vector<cv::Point> obj = blobs[marker_index[j]];
		for (int i = 0; i < obj.size(); i++) {
			int x = obj[i].x;
			int y = obj[i].y;
			output2.at<char>(y, x) = 255;
		}
	}
	

	return output2;
}