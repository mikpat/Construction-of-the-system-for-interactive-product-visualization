#include "display.h"


void drawCAMchar(contourCharacteristics cam_char, Mat src) {

	namedWindow("Camera", WINDOW_NORMAL);
	line(src, cam_char.max_1, cam_char.max_2, Scalar(0, 0, 250), 2, 8, 0);
	circle(src, cam_char.center_of_mass, 4, Scalar(255, 0, 0), CV_FILLED, 8, 0);
	imshow("Camera", src);

}

//shift cad characteristics to show them on the screen
void drawCADchar(vector<Point> contour, contourCharacteristics cad_char) {
	cad_char.max_1.x /= 100;
	cad_char.max_2.x /= 100;
	cad_char.min_1.x /= 100;
	cad_char.min_2.x /= 100;
	cad_char.max_1.y /= 100;
	cad_char.max_2.y /= 100;
	cad_char.min_1.y /= 100;
	cad_char.min_2.y /= 100;
	cad_char.intersection.x /= 100;
	cad_char.intersection.y /= 100;

	cad_char.max_1.x += 200;
	cad_char.max_2.x += 200;
	cad_char.min_1.x += 200;
	cad_char.min_2.x += 200;
	cad_char.max_1.y += 200;
	cad_char.max_2.y += 200;
	cad_char.min_1.y += 200;
	cad_char.min_2.y += 200;
	cad_char.intersection.x += 200;
	cad_char.intersection.y += 200;

	Mat drawing = Mat::zeros(600, 600, CV_8UC3);
	namedWindow("Model", WINDOW_NORMAL);
	line(drawing, cad_char.max_1, cad_char.max_2, Scalar(250, 0, 0), 3, 8, 0);
	line(drawing, cad_char.min_1, cad_char.min_2, Scalar(250, 0, 0), 3, 8, 0);
	circle(drawing, cad_char.intersection, 3, Scalar(255, 0, 0), CV_FILLED, 8, 0);
	for (int i = 0; i < contour.size(); i++)
	{
		contour[i].x += 200;
		contour[i].y += 200;
		circle(drawing, contour[i], 3, Scalar(0, 0, 255), CV_FILLED, 6, 0);
	}
	imshow("Model", drawing);
	waitKey(0);
}


void displayTmatrix(float(*matrix)[3]) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			printf("%0.3f ", matrix[i][j]);
		printf("\n");
	}
}