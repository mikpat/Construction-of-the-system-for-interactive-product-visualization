#include "cam_source.h"

int thresh = 1;
int approxValue = 2;
vector<vector<Point> > contours_src;
Mat cam_src;

//initial values for threshholding
struct hsv_thresh {
	int hmx = 255;
	int hmn = 0;
	int smx = 255;
	int smn = 0;
	int vmx = 255;
	int vmn = 130;
}init_values;

Mat getCAMsrc() {
	return cam_src;
}


contourCharacteristics camCapture(bool cameraCalibration) {
	
	contourCharacteristics char_line;
	

	//set projection as white image
	Mat cam_init_projection;
	cam_init_projection = cv::imread("../data/projection_black.png", CV_LOAD_IMAGE_COLOR);
	cvNamedWindow("CamInit", CV_WINDOW_NORMAL);
	cvSetWindowProperty("CamInit", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	imshow("CamInit", cam_init_projection);
	cv::waitKey(100);
	
	cv::VideoCapture cap(0); // open the default camera


	cap >> cam_src;
	cam_src = cam_src(cv::Rect(120, 0, 520, 480));
	cv::destroyWindow("CamInit");

	Mat cam_input = cam_src;


	
	if (!cam_src.data) {
		char_line.error = true;
		return char_line;
	}
	else if (cameraCalibration == 1) {
		namedWindow("Calibration", WINDOW_NORMAL);
		Mat image=cam_src;
		int c = 0, xCorrection=0, yCorrection=0; 
		circle(image, Point(xCorrection, yCorrection), 1, Scalar(0, 0, 255), CV_FILLED, 6, 0);
		imshow("Calibration", image);
		cv::waitKey(10);
		do
		{
			c = 0;
			switch ((c = _getch())) {
			case KEY_UP:
				yCorrection -= 1;
				image = cam_src;
				circle(image, Point(xCorrection, yCorrection), 1, Scalar(0, 0, 255), CV_FILLED, 6, 0);
				imshow("Calibration", image);
				cv::waitKey(10);
				break;
			case KEY_DOWN:
				yCorrection += 1;
				image = cam_src;
				circle(image, Point(xCorrection, yCorrection), 1, Scalar(0, 0, 255), CV_FILLED, 6, 0);
				imshow("Calibration", image);
				cv::waitKey(10);
				break;
			case KEY_LEFT:
				xCorrection -= 1;
				image = cam_src;
				circle(image, Point(xCorrection, yCorrection), 1, Scalar(0, 0, 255), CV_FILLED, 6, 0);
				imshow("Calibration", image);
				cv::waitKey(10);
				break;
			case KEY_RIGHT:
				xCorrection +=1;
				image = cam_src;
				circle(image, Point(xCorrection, yCorrection), 1, Scalar(0, 0, 255), CV_FILLED, 6, 0);
				imshow("Calibration", image);
				cv::waitKey(10);
				break;
			default:
				break;
			}
		}
		while (c != KEY_ESC);

		float rotCorrection = 0;
		c = 0;
		image = cam_src;
		Mat rot_mat;
		cv::Point2f src_center(yCorrection, xCorrection);
		do {
			switch ((c = _getch())) {
			case KEY_UP:
				rotCorrection++;
				rot_mat = getRotationMatrix2D(src_center, 1.0, 1.0);
				warpAffine(image, image, rot_mat, image.size());
				imshow("Calibration", image);
				cv::waitKey(10);
				break;
			case KEY_DOWN:
				rotCorrection--;
				rot_mat = getRotationMatrix2D(src_center, -1.0, 1.0);
				warpAffine(image, image, rot_mat, image.size());
				imshow("Calibration", image);
				cv::waitKey(10);
				break;
			default:
				break;
			}
		} while (c != KEY_ESC);
		rotCorrection *= 0.01745;

		ofstream report("../data/calibrationCamera.txt");
		if (report.is_open())
		{
			report << "" << (yCorrection) << endl;
			report << "" << (xCorrection) << endl;
			report << "" << (rotCorrection) << endl;
			report << "First value corresponds to Y coordinate correction, second to X " << endl;
		}
		else
			cout << "\n\nUnable to open callibration report\n\n";
	}
	else {
		Mat binary_image;
		contours_src = morphology_trans(binary_image);
		cv::Moments m = cv::moments(binary_image, true);
		char_line.center_of_mass = Point(m.m10 / m.m00, m.m01 / m.m00);

		getCharacteristics(contours_src, char_line);

		return char_line;
	}
}

vector<vector<Point> > morphology_trans(Mat &binary_output) {
	Mat threshold, HSV, hthresh, sthresh, vthresh, tracking;
	Mat image = cam_src;
	vector<Mat> hsvChannels(3);
	cvtColor(image, HSV, cv::COLOR_BGR2HSV);
	split(HSV, hsvChannels);

	inRange(hsvChannels[2], init_values.vmn, init_values.vmx, tracking);

	namedWindow("tracking", WINDOW_NORMAL);
	imshow("tracking", tracking);
	cv::waitKey(0);
	Mat dilateSrc, morphologySrc, gaussSrc;
	dilate(tracking, dilateSrc, Mat(), Point(-1, -1), 4);
	morphologyEx(dilateSrc, morphologySrc, cv::MORPH_CLOSE, Mat(), Point(-1, -1), 3);
	cv::threshold(morphologySrc, morphologySrc, 0.0, 1.0, cv::THRESH_BINARY);
	Mat objectDetection = findObjects(morphologySrc);
	binary_output = objectDetection;

	GaussianBlur(objectDetection, gaussSrc, Size(3, 3), 2, 2);
	namedWindow("prog", WINDOW_NORMAL);
	imshow("prog", gaussSrc);
	cv::waitKey(0);


	//find approximated contours using Canny and findContours
	vector<vector<Point> > contours;
	approx_counturs(gaussSrc, contours);

	return contours;
}

void getCharacteristics(vector<vector<Point> > &contours, contourCharacteristics &var) {

	max_line(contours, var);
	min_line(contours, var);
}

void getCharacteristics(vector<Point> &contours, contourCharacteristics &var) {

	max_line(contours, var);
	min_line(contours, var);
}

void max_line(vector<Point> contour, contourCharacteristics &var) {

	int max_dis = 0, current_dis;
	for (int i = 0; i < contour.size(); i++) {
		for (int j = 0; j < (contour.size() - i - 1); j++) {
			current_dis = euclideanDist(contour[i], contour[j]);
			if (current_dis > max_dis) {
				max_dis = current_dis;
				var.max_1 = contour[i];
				var.max_2 = contour[j];
			}
		}
	}
}

void min_line(vector<Point> contour, contourCharacteristics &var) {
	int min_dis = euclideanDist(var.max_1, var.max_2), current_dis;
	for(int i = 0; i < contour.size(); i++){
		for(int j = 0; j < contour.size(); j++){
			if (i == j)
				continue;
			current_dis = euclideanDist(contour[i], contour[j]);
			if ((current_dis < min_dis) && (current_dis > 0.32*euclideanDist(var.max_1, var.max_2)) && crossMaxLine(contour[i], contour[j], var)) {
				min_dis = current_dis;
				var.min_1 = contour[i];
				var.min_2 = contour[j];
			}
		}
	}
	
	var.intersection = get_line_intersection(var.max_1.x, var.max_1.y, var.max_2.x, var.max_2.y, var.min_1.x, var.min_1.y, var.min_2.x, var.min_2.y);
}


void max_line(vector<vector<Point> > contours, contourCharacteristics &var) {

	int max_dis = 0, current_dis;
	for (int i = 0; i < contours.size(); i++) {
		for (int j = 0; j < contours[i].size(); j++) {
			for (int n = 0; n < (contours.size()); n++) {
				for (int m = 0; m < contours[n].size(); m++) {
					current_dis = euclideanDist(contours[i][j], contours[n][m]);
					if (current_dis > max_dis) {
						max_dis = current_dis;
						var.max_1 = contours[i][j];
						var.max_2 = contours[n][m];
					}
				}
			}
		}
	}
}

void min_line(vector<vector<Point> > contours, contourCharacteristics &var) {
	int min_dis = euclideanDist(var.max_1, var.max_2), current_dis;
	for (int i = 0; i < contours.size(); i++) {
		for (int j = 0; j < contours[i].size(); j++) {
			for (int n = 0; n < (contours.size()); n++) {
				for (int m = 0; m < contours[n].size(); m++) {
					current_dis = euclideanDist(contours[i][j], contours[n][m]);
					if ((current_dis < min_dis) && (current_dis > (0.32*euclideanDist(var.max_1, var.max_2))) && crossMaxLine(contours[i][j], contours[n][m], var)) {
						min_dis = current_dis;
						var.min_1 = contours[i][j];
						var.min_2 = contours[n][m];
					}
				}
			}
		}
	}
	var.intersection = get_line_intersection(var.max_1.x, var.max_1.y, var.max_2.x, var.max_2.y, var.min_1.x, var.min_1.y, var.min_2.x, var.min_2.y);
}

Point get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
	float p2_x, float p2_y, float p3_x, float p3_y)
{
	float s1_x, s1_y, s2_x, s2_y;
	s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
	s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

	float s, t;
	s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
	t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

	if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
	{
		// Collision detected
		Point returnValue;
		returnValue.x = (p0_x + (t * s1_x));
		returnValue.y = (p0_y + (t * s1_y));
		return returnValue;
	}

	Point returnValue = (0, 0);
	return returnValue; // No collision
}

bool crossMaxLine(Point m1, Point m2, contourCharacteristics &var) {
	
	float max1_y, max2_y, max1_x, max2_x;
	if (var.max_1.y >= var.max_2.y) {
		max2_y = var.max_1.y;
		max1_y = var.max_2.y;
	}
	else {
		max2_y = var.max_2.y;
		max1_y = var.max_1.y;
	}
	if (var.max_1.x >= var.max_2.x) {
		max2_x = var.max_1.x;
		max1_x = var.max_2.x;
	}
	else {
		max2_x = var.max_2.x;
		max1_x = var.max_1.x;
	}
	Point m3 = var.max_1;
	Point m4 = var.max_2;
	Point intersection = get_line_intersection(var.max_1.x, var.max_1.y, var.max_2.x, var.max_2.y, m1.x, m1.y, m2.x, m2.y);
	if (intersection.x == 0 && intersection.y == 0)
		return 0;

	float dmaxX = max2_x - max1_x;
	float dmaxY = max2_y - max1_y;
	float scale = 0.3;
	if (((intersection.x > (max1_x + scale*dmaxX)) && (intersection.x < (max2_x + scale*dmaxX)))
		&& ((intersection.y >(max1_y + scale*dmaxY)) && (intersection.y < (max2_y + scale*dmaxY))))
	{
		return true;
	}
	else
		return false;
}


int euclideanDist(Point& p, Point& q) {
	Point d = p - q;
	return (int)sqrt(d.x*d.x + d.y*d.y);
}


void thresh_callback(int, void*)
{
	Mat threshold, HSV, hthresh, sthresh, vthresh, tracking;
	vector<Mat> hsvChannels(3);
	cvtColor(cam_src, HSV, cv::COLOR_BGR2HSV);
	split(HSV, hsvChannels);

	int hmn, hmx, smn, smx, vmn, vmx;
	hmn = getTrackbarPos("hmin", "HueComp");
	hmx = getTrackbarPos("hmax", "HueComp");

	smn = getTrackbarPos("smin", "SatComp");
	smx = getTrackbarPos("smax", "SatComp");

	vmn = getTrackbarPos("vmin", "ValComp");
	vmx = getTrackbarPos("vmax", "ValComp");

	inRange(hsvChannels[0], hmn, hmx, hthresh);
	inRange(hsvChannels[1], smn, smx, sthresh);
	inRange(hsvChannels[2], vmn, vmx, vthresh);

	bitwise_and(sthresh, vthresh, tracking);
	bitwise_and(hthresh, tracking, tracking);

	imshow("HueComp", hthresh);
	imshow("SatComp", sthresh);
	imshow("ValComp", vthresh);

	Mat dilateSrc, morphologySrc, gaussSrc;
	dilate(tracking, dilateSrc, Mat(), Point(-1, -1), 4);
	morphologyEx(dilateSrc, morphologySrc, cv::MORPH_CLOSE, Mat(), Point(-1, -1), 2);
	GaussianBlur(morphologySrc, gaussSrc, Size(3, 3), 2, 2);

	imshow("Tracking", tracking);
	namedWindow("GaussianBlur", WINDOW_NORMAL);
	imshow("GaussianBlur", gaussSrc);


}


void approx_counturs(Mat image, vector<vector<Point> > &contours) {
	Mat canny_output;
	vector<Vec4i> hierarchy;
	Canny(image, canny_output, thresh, thresh * 2, 3);
	findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	//approximate polynomial for less computation
	for (size_t i = 0; i< contours.size(); i++)
		approxPolyDP(contours[i], contours[i], approxValue, false);
	//drawContours(canny_output.size(), contours, hierarchy);
}

void drawContours(Size dimensions, vector<vector<Point> >&contours, vector<Vec4i> &hierarchy) {
	//draw contours
	Mat drawing = cam_src;
	for (size_t i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(250, 250, 250);
		drawContours(drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point());
	}
	namedWindow("Contours", WINDOW_NORMAL);
	imshow("Contours", drawing);
}


Mat findObjects(Mat scr_image) {

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

	int biggest_index=0;
	int biggestObj = 0;

	for (int i = 0; i < blobs.size(); i++) {
		if ( blobs[i].size() > biggestObj) {
			biggest_index = i;
			biggestObj = blobs[i].size();
		}
	}		
	Mat output = Mat::zeros(scr_image.size(), CV_8UC1);
	vector<Point> obj = blobs[biggest_index];
	size_t i;
	for (i = 0; i < obj.size(); i++) {
		int x = obj[i].x;
		int y = obj[i].y;
		output.at<char>(y, x) = 255;
	}
	return output;
}
