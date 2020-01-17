#pragma once

#include <stdio.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <conio.h>
#include <iostream>
#include <fstream>

#define KEY_UP 72
#define KEY_DOWN 80
#define KEY_LEFT 75
#define KEY_RIGHT 77
#define KEY_ESC 27

using namespace std;

using cv::Mat;
using cv::Point;
using cv::Size;
using cv::Vec4i;
using cv::getTrackbarPos;
using cv::WINDOW_NORMAL;
using cv::WINDOW_AUTOSIZE;
using cv::namedWindow;
using cv::Scalar;
using cv::createTrackbar;
Mat gaussSrc;

//holds 4 points defining characteristic lines of shoe
struct contourCharacteristics {
	cv::Point max_1;
	cv::Point max_2;
	cv::Point min_1;
	cv::Point min_2;
	cv::Point intersection;
	cv::Point center_of_mass;
	bool error=0;
};


contourCharacteristics camCapture(bool);
Mat findObjects(Mat image);
Mat getCAMsrc();
void getCharacteristics(vector<vector<Point> > &, contourCharacteristics &);
void getCharacteristics(vector<Point> &, contourCharacteristics &);
vector<vector<Point> > morphology_trans(Mat &);
void thresh_callback(int, void*);
void eliminate_contours(Mat);
void max_line(vector<vector<Point> >, contourCharacteristics &);
void min_line(vector<vector<Point> >, contourCharacteristics &);
void max_line(vector<Point>, contourCharacteristics &);
void min_line(vector<Point>, contourCharacteristics &);
bool crossMaxLine(Point, Point, contourCharacteristics &);
void approx_counturs(Mat, vector<vector<Point> >&);
int euclideanDist(Point& p, Point& q);
cv::Point get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
	float p2_x, float p2_y, float p3_x, float p3_y);
void drawContours(Size, vector<vector<Point> >&, vector<Vec4i> &);
