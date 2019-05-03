#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
//#include "features2d.hpp"
#include<iostream>
#include<string.h>
#include<sstream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "tserial.h"
#include "bot_control.h"
#include <fstream>

#define PI 3.14159265

using namespace cv;
using namespace std;

Mat morph_operations(Mat src, Mat dst, Size kernel_size, int center, int operation);
void bwareOpen(Mat* image, double size);
double axisProperties(vector<Point> contour);



int main()
{
	Mat img, imgOriginal, imgGray, imgThreshold, imgOpen, imgClose, imgHsv, imgHsvThreshold, imgSilver, imgYellow, silver, yellow;
	Mat other, other2;
	vector<cv::Mat> values;
	vector<vector<Point>> contours, contours2, contours3;
	vector <Point> shapeBolt(44);
	vector <Point> shapeNut(28);

	int countBolt = 0;
	int ind = 0;
	vector<Vec4i> hierarchy, hierarchy2, hierarchy3;
	Mat grad;
	int scale = 1;
	int deltaEdge = 0;
	int ddepth = CV_16S;
	int process = 0;

	double x, y = 0.5;
	double z = 0.5;

	char flag = 'a';

	char * data = (char*)'v';
	char * dataOld = (char*)'u';

	serial comm;
	stringstream s;
	string result;

	VideoCapture cap(0);

	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<double>(0, 0) = 6.0486080554129342e+002;
	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(0, 2) = 3.1950000000000000e+002;

	cameraMatrix.at<double>(1, 0) = 0;
	cameraMatrix.at<double>(1, 1) = 6.0486080554129342e+002;
	cameraMatrix.at<double>(1, 2) = 2.3950000000000000e+002;

	cameraMatrix.at<double>(2, 0) = 0;
	cameraMatrix.at<double>(2, 1) = 0;
	cameraMatrix.at<double>(2, 2) = 1;


	Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
	distCoeffs.at<double>(0, 0) = 8.2620139698452666e-002;
	distCoeffs.at<double>(1, 0) = -2.7675886003881384e-001;
	distCoeffs.at<double>(3, 0) = 0;
	distCoeffs.at<double>(4, 0) = 0;
	distCoeffs.at<double>(5, 0) = 5.9528994991108919e-001;


	cap.set(CV_CAP_PROP_SETTINGS, 1);
	cap.set(CV_CAP_PROP_FOCUS, 0);
	cap.set(CV_CAP_PROP_BRIGHTNESS, 30);
	cap.set(CV_CAP_PROP_SATURATION, 200);
	cap.set(CV_CAP_PROP_EXPOSURE, -7); //-7
	cap.set(CV_CAP_PROP_CONTRAST, 10);

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "nao vai dar nao" << endl;
		return -1;
	}

	int YellowLowH = 20; // 25;
	int YellowHighH = 40; // 57;

	int YellowLowS = 213; // 115;
	int YellowHighS = 255; // 255;

	int YellowLowV = 170;// 132;
	int YellowHighV = 255; // 255;

	int SilverLowH = 0;// 51; //0
	int SilverHighH = 179; //179

	int SilverLowS = 0; //0
	int SilverHighS = 121;// 117; //121

	int SilverLowV = 121;//136; //121
	int SilverHighV = 255; //255

	//namedWindow("Control", CV_WINDOW_NORMAL); //create a window called "Control"

	cvCreateTrackbar("LowH", "Control", &YellowLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &YellowHighH, 179);

	cvCreateTrackbar("LowS", "Control", &YellowLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &YellowHighS, 255);

	cvCreateTrackbar("LowV", "Control", &YellowLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &YellowHighV, 255);


	shapeBolt[0] = Point(246, 85);
	shapeBolt[1] = Point(245, 86);
	shapeBolt[2] = Point(243, 86);
	shapeBolt[3] = Point(242, 87);
	shapeBolt[4] = Point(241, 87);
	shapeBolt[5] = Point(239, 89);
	shapeBolt[6] = Point(239, 91);
	shapeBolt[7] = Point(240, 92);
	shapeBolt[8] = Point(240, 94);
	shapeBolt[9] = Point(241, 94);
	shapeBolt[10] = Point(242, 95);
	shapeBolt[11] = Point(243, 95);
	shapeBolt[12] = Point(244, 96);
	shapeBolt[13] = Point(244, 100);
	shapeBolt[14] = Point(245, 101);
	shapeBolt[15] = Point(245, 103);
	shapeBolt[16] = Point(246, 104);
	shapeBolt[17] = Point(246, 108);
	shapeBolt[18] = Point(247, 109);
	shapeBolt[19] = Point(247, 116);
	shapeBolt[20] = Point(248, 117);
	shapeBolt[21] = Point(248, 121);
	shapeBolt[22] = Point(249, 122);
	shapeBolt[23] = Point(249, 127);
	shapeBolt[24] = Point(250, 128);
	shapeBolt[25] = Point(250, 129);
	shapeBolt[26] = Point(253, 132);
	shapeBolt[27] = Point(256, 132);
	shapeBolt[28] = Point(257, 131);
	shapeBolt[29] = Point(258, 131);
	shapeBolt[30] = Point(258, 130);
	shapeBolt[31] = Point(260, 128);
	shapeBolt[32] = Point(260, 122);
	shapeBolt[33] = Point(259, 121);
	shapeBolt[34] = Point(259, 115);
	shapeBolt[35] = Point(258, 114);
	shapeBolt[36] = Point(258, 110);
	shapeBolt[37] = Point(257, 109);
	shapeBolt[38] = Point(257, 102);
	shapeBolt[39] = Point(256, 101);
	shapeBolt[40] = Point(256, 94);
	shapeBolt[41] = Point(258, 92);
	shapeBolt[42] = Point(258, 86);
	shapeBolt[43] = Point(257, 85);

	shapeNut[0] = Point(247, 119);
	shapeNut[1] = Point(245, 121);
	shapeNut[2] = Point(245, 122);
	shapeNut[3] = Point(244, 123);
	shapeNut[4] = Point(244, 124);
	shapeNut[5] = Point(243, 125);
	shapeNut[6] = Point(243, 131);
	shapeNut[7] = Point(244, 131);
	shapeNut[8] = Point(245, 132);
	shapeNut[9] = Point(245, 134);
	shapeNut[10] = Point(246, 133);
	shapeNut[11] = Point(248, 135);
	shapeNut[12] = Point(249, 135);
	shapeNut[13] = Point(250, 136);
	shapeNut[14] = Point(251, 135);
	shapeNut[15] = Point(252, 135);
	shapeNut[16] = Point(254, 133);
	shapeNut[17] = Point(254, 131);
	shapeNut[18] = Point(251, 128);
	shapeNut[19] = Point(252, 127);
	shapeNut[20] = Point(253, 127);
	shapeNut[21] = Point(254, 126);
	shapeNut[22] = Point(255, 126);
	shapeNut[23] = Point(257, 124);
	shapeNut[24] = Point(257, 123);
	shapeNut[25] = Point(258, 122);
	shapeNut[26] = Point(258, 121);
	shapeNut[27] = Point(256, 119);

	while (true)
	{
		comm.startDevice("COM3", 9600);
		if (comm.get_data() == 'w') {
		//if(true){
			/*int col = 640;
			int lin = 480;
			int x1 = 0, y1 = 0;
			Rect roi(x1, y1, col, lin);*/
			int col = 640;
			int lin = 480;
			int x1 = 60, y1 = 0;
			Rect roi(x1, y1, col - 150, lin - 200);
			bool bSuccess = 1;
			Point2d aux;
			int object = 0;
			Mat points = Mat::zeros(3, 1, CV_64F);
			double theta2, theta1, angle;

			//vector<Point> shapeBolt, ShapeNut;

			String res;
			ostringstream txt;

			cap.read(img);

			undistort(img, imgOriginal, cameraMatrix, distCoeffs, cameraMatrix);

			if (!bSuccess) //if not success, break loop
			{
				cout << "Nao vai dar nao" << endl;
				break;
			}

			imgOriginal = imgOriginal(roi);
			flip(imgOriginal, imgOriginal, -1);

			Mat imgEdge = imgOriginal;
			//imshow("Trapezio descendente!", imgOriginal);
			GaussianBlur(imgEdge, imgEdge, Size(3, 3), 0, 0, BORDER_DEFAULT);
			/// Convert it to gray
			cvtColor(imgEdge, imgGray, CV_BGR2GRAY);
			/// Generate grad_x and grad_y
			Mat grad_x, grad_y;
			Mat abs_grad_x, abs_grad_y;
			/// Gradient X
			//Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
			Sobel(imgGray, grad_x, ddepth, 1, 0, 3, scale, deltaEdge, BORDER_DEFAULT);
			convertScaleAbs(grad_x, abs_grad_x);
			/// Gradient Y
			//Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
			Sobel(imgGray, grad_y, ddepth, 0, 1, 3, scale, deltaEdge, BORDER_DEFAULT);
			convertScaleAbs(grad_y, abs_grad_y);
			/// Total Gradient (approximate)
			addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
			//imshow("Hora do show,porra!", grad);

			threshold(grad, imgThreshold, 50, 255, 1);
			bitwise_not(imgThreshold, imgThreshold);


			//imshow("Threshold", imgThreshold);

			Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
			morphologyEx(imgThreshold, imgOpen, MORPH_OPEN, element);
			morphologyEx(imgOpen, imgClose, MORPH_CLOSE, element, Point(-1, -1), 3);
			dilate(imgClose, imgClose, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)));



			findContours(imgClose.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

			for (int i = 0; i < contours.size(); i++)
			{
				// Calculate contour area
				double area = contourArea(contours[i]);

				// Remove small objects by drawing the contour with black color
				if (area > 0 && area <= 230)
					drawContours(imgClose, contours, i, CV_RGB(0, 0, 0), -1);
			}
			threshold(imgClose, imgClose, 0, 255, 0);
			//imshow("All Atoms", imgClose);

			//Color Threshold ------------------------------------------------------------------------------------------------------
			blur(imgOriginal, imgOriginal, Size(3, 3), Point(-1, -1));
			cvtColor(imgOriginal, imgHsv, COLOR_BGR2HSV);
			split(imgHsv, values);

			equalizeHist(values[1], values[1]);

			merge(values, imgHsv);
			//Yellow
			inRange(imgHsv, Scalar(YellowLowH, YellowLowS, YellowLowV), Scalar(YellowHighH, YellowHighS, YellowHighV), imgHsvThreshold);
			//imshow("Color Threshold", imgHsvThreshold);

			dilate(imgHsvThreshold, imgHsvThreshold, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));
			dilate(imgHsvThreshold, imgHsvThreshold, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
			//imshow("Aqui a gente constroi fibra", imgHsvThreshold);

			findContours(imgHsvThreshold.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

			for (int i = 0; i < contours.size(); i++)
			{
				// Calculate contour area
				double area = contourArea(contours[i]);

				// Remove small objects by drawing the contour with black color
				if (area > 0 && area <= 200)
					drawContours(imgHsvThreshold, contours, i, CV_RGB(0, 0, 0), -1);
			}
			bitwise_and(imgClose, imgHsvThreshold, imgYellow);
			imshow("Image Yellow", imgYellow);

			//Silver
			inRange(imgHsv, Scalar(SilverLowH, SilverLowS, SilverLowV), Scalar(SilverHighH, SilverHighS, SilverHighV), silver);

			dilate(silver, silver, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));
			dilate(silver, silver, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
			//imshow("Silver", silver);

			findContours(silver.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

			for (int i = 0; i < contours.size(); i++)
			{
				// Calculate contour area
				double area = contourArea(contours[i]);

				// Remove small objects by drawing the contour with black color
				if (area > 0 && area <= 230)
					drawContours(silver, contours, i, CV_RGB(0, 0, 0), -1);
			}
			threshold(silver, imgSilver, 0, 255, 0);
			imshow("Silver", imgSilver);

			// OTHER OBJECTS PICKUP ---------------------------------------------------------------------------------------------------
			imshow("imgClose", imgClose);
			findContours(imgClose.clone(), contours3, hierarchy3, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
			vector<Point2d> massCenter3(contours3.size());
			vector<Moments> moment3(contours3.size());

			for (int i = 0; i < contours3.size(); i++)
			{
				moment3[i] = moments(contours3[i], false);
				massCenter3[i] = Point2d(moment3[i].m10 / moment3[i].m00, moment3[i].m01 / moment3[i].m00);
			}


			if (!contours3.empty() && !hierarchy3.empty())
			{

				int idx = 0;
				for (; idx >= 0; idx = hierarchy3[idx][0])
				{

					double compareBolt = matchShapes(contours3[idx], shapeBolt, CV_CONTOURS_MATCH_I3, 0.0);
					double compareNut = matchShapes(contours3[idx], shapeNut, CV_CONTOURS_MATCH_I3, 0.0);

					Point2f circleCenter;
					float circleRadius;
					minEnclosingCircle(contours3[idx], circleCenter, circleRadius);

					if ((compareNut > 0.55 && compareBolt > 0.5) && (arcLength(contours3[idx], true) >= 190) && axisProperties(contours3[idx]) > 2)
					{

						aux.x = 0;
						aux.y = 0;

						aux.x = (-1 * (massCenter3[idx].x - 222));
						aux.y = ((massCenter3[idx].y) + 40);
						//if (aux.y < 255)
						//{

						string tests = to_string(axisProperties(contours3[idx]));

						putText(imgOriginal, "Other", massCenter3[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 255), 2);


						if (contours3[idx].size() >= 5)
						{
							RotatedRect rec = fitEllipse(contours3[idx]);
							theta1 = atan(aux.y / aux.x) * 180 / PI;
							theta2 = (rec.angle);
							angle = theta1 - (90 - theta2);
							object = 5;
						}


					}
				}
			}
			//------------------------------------------------------------------------------------------------------------------------------


			// YELLOW OBJECT PICKUP ---------------------------------------------------------------------------------------------------
			threshold(imgYellow, yellow, 0, 255, 0);
			findContours(imgYellow.clone(), contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

			vector<Point2d> massCenter(contours.size());
			vector<Moments> moment(contours.size());

			for (int i = 0; i < contours.size(); i++)
			{
				moment[i] = moments(contours[i], false);
				massCenter[i] = Point2d(moment[i].m10 / moment[i].m00, moment[i].m01 / moment[i].m00);
			}

			if (!contours.empty() && !hierarchy.empty())
			{


				aux = Point2d(0, 0);
				// iterate through all the top-level contours,
				// draw each connected component with its own random color
				int idx = 0;
				for (; idx >= 0; idx = hierarchy[idx][0])
				{

					aux.x = (-1 * (massCenter[idx].x - 222));
					aux.y = ((massCenter[idx].y) + 40);

					string xy = "[" + to_string(cvRound(aux.x)) + ", " + to_string(cvRound(aux.y)) + "]";


					Point2f circleCenter;
					float circleRadius;
					minEnclosingCircle(contours[idx], circleCenter, circleRadius);

					string test1 = to_string(cvRound(PI*circleRadius*circleRadius)) + "; " + to_string(axisProperties(contours[idx]));
					//putText(imgOriginal, test1, massCenter[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255, 255), 2);

					if (PI*circleRadius*circleRadius > 1000 && axisProperties(contours[idx]) < 2)
					{
						putText(imgOriginal, "Overlaped", massCenter[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 255), 2);
						object = 6;
					}
					else if ((axisProperties(contours[idx]) >= 1.80) && (axisProperties(contours[idx]) < 10) && (arcLength(contours[idx], true) >= 100) && (arcLength(contours[idx], true) <= 190))
					{
						RotatedRect rec = fitEllipse(contours[idx]);
						theta1 = atan(aux.y / aux.x) * 180 / PI;
						theta2 = (rec.angle);
						angle = theta1 - (90 - theta2);

						string test = to_string(cvRound(theta1)) + "," + to_string(cvRound(theta2)) + "," + to_string(cvRound(angle));

						putText(imgOriginal, "Yellow Bolt", massCenter[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 255), 2);
						object = 1;//Yellow Bolt
					}
					else if ((arcLength(contours[idx], true) < 290) && (axisProperties(contours[idx]) < 1.3))
					{

						putText(imgOriginal, "Yellow Nut", massCenter[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 255), 2);
						object = 2;//Yellow Nut
						angle = 0;
					}
					else
					{
						putText(imgOriginal, "Unknown Yellow Object", massCenter[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 255), 2);
						object = 6;
					}

				}

			}


			// SILVER OBJECT PICKUP ---------------------------------------------------------------------------------------------------
			findContours(imgSilver.clone(), contours2, hierarchy2, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

			vector<Point2d> massCenter2(contours2.size());
			vector<Moments> moment2(contours2.size());


			for (int i = 0; i < contours2.size(); i++)
			{
				moment2[i] = moments(contours2[i], false);
				massCenter2[i] = Point2d(moment2[i].m10 / moment2[i].m00, moment2[i].m01 / moment2[i].m00);

			}


			if (!contours2.empty() && !hierarchy2.empty())
			{
				aux = Point2d(0, 0);

				int idx = 0;
				for (; idx >= 0; idx = hierarchy2[idx][0])
				{
					aux.x = 0;
					aux.y = 0;

					aux.x = (-1 * (massCenter2[idx].x - 222));
					aux.y = ((massCenter2[idx].y) + 40);

					string xy = "[" + to_string(cvRound(aux.x)) + ", " + to_string(cvRound(aux.y)) + "]";

					Point2f circleCenter;
					float circleRadius;
					minEnclosingCircle(contours2[idx], circleCenter, circleRadius);

					if (PI*circleRadius*circleRadius > 1000 && axisProperties(contours2[idx]) < 1.5)
					{
						putText(imgOriginal, "Overlaped", massCenter2[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 255), 2);
						object = 6;
					}
					else if ((axisProperties(contours2[idx]) >= 2.50) && (axisProperties(contours2[idx]) < 10) && (arcLength(contours2[idx], true) >= 100) && (arcLength(contours2[idx], true) <= 190))
					{

						RotatedRect rec = fitEllipse(contours2[idx]);
						theta1 = atan(aux.y / aux.x) * 180 / PI;
						theta2 = (rec.angle);
						angle = theta1 - (90 - theta2);

						string test = to_string(cvRound(theta1)) + "," + to_string(cvRound(theta2)) + "," + to_string(cvRound(angle));


						putText(imgOriginal, "Silver Bolt", massCenter2[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 255), 2);
						object = 3;//SilverBolt;	

					}
					else if ((arcLength(contours2[idx], true) < 290) && (axisProperties(contours2[idx]) < 1.3))
					{

						putText(imgOriginal, "Silver Nut", massCenter2[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 255), 2);
						object = 4;//Silver nut
						angle = 0;
					}
					else
					{
						putText(imgOriginal, "Unknow Silver Object", massCenter2[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 255), 2);
						object = 6;
					}

				}

			}

			
			if (process == 4)
			{
				cout << "Object" << object << endl;
				s.str("");
				s.clear();
				//Compensation for camera distortion
				angle = cvRound(angle);
				if (aux.x < 0 && aux.y < 180) //1
					s << cvRound(aux.x*0.81+5) << "," << cvRound(aux.y-5) << "," << object << "," << angle << ",";
				else if (aux.x > 0 && aux.y < 180) //2
					s << cvRound(aux.x*0.8) << "," << cvRound(aux.y*1.05) << "," << object << "," << angle << ",";
				else if (aux.x > 0 && aux.y > 180) //3
					s << cvRound(aux.x*0.85) << "," << cvRound(aux.y*1.04) << "," << object << "," << angle << ",";
				else if (aux.x < 0 && aux.y > 180) //4
					s << cvRound(aux.x*0.90) << "," << cvRound(aux.y) << "," << object << "," << angle << ",";
				result = s.str();
				data = (char*)result.c_str();
				printf("[%s]\n", data);
				int l = 0;
				cap.read(img);
				while (data[l] != '\0')
				{
					comm.send_data(data[l]);
					l++;
				}
				process = 0;
				imshow("Final", imgOriginal);
				object = 0;
			}
			process++;

		}//endIF


		comm.stopDevice();
		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}

	}

	return 0;
}
Mat morph_operations(Mat src, Mat dst, Size kernel_size, int center, int operation)
{
	Mat element = getStructuringElement(MORPH_RECT, kernel_size, Point(center, center));

	morphologyEx(src, dst, operation, element);

	return dst;
}
double axisProperties(vector<Point> contour)
{
	float majorAxis, minorAxis;
	Point center;
	if (contour.size() >= 5)
	{

		RotatedRect minEllipse = fitEllipse(contour);
		majorAxis = max(minEllipse.size.height, minEllipse.size.width);
		minorAxis = min(minEllipse.size.height, minEllipse.size.width);

		return (majorAxis / minorAxis);
	}

}
void bwareOpen(Mat* image, double size)
{
	vector<vector<Point>> contours;

	findContours(image->clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	for (int i = 0; i < contours.size(); i++)
	{
		// Calculate contour area
		double area = contourArea(contours[i]);

		// Remove small objects by drawing the contour with black color
		if (area > 0 && area <= size)
			drawContours(*image, contours, i, CV_RGB(0, 0, 0), -1);
	}

	return;
}
