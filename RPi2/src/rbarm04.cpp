#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <thread>
#include <string>
#include <sstream>
#include <stdio.h>
#include <gtk/gtk.h>
#include <chrono>
#include "bot_control.h"
#include <fstream>

#define PI 3.14159265

using namespace cv;
using namespace std;

//-----------------------Global Vars---------------------------------------------------
Mat cameraMatrix = Mat::eye(3,3,CV_64F); Mat distCoeffs = Mat::zeros(5,1,CV_64F);
vector <Point> shapeBolt2(25); vector <Point> shapeBolt1(36);
VideoCapture cap; 
Point p, pkup;
int loopCount = 0; int object = 0;
double OrtAng;
int pickup = 0;
int autoM = 0;

serial comm; bool DeviceCheck = comm.startDevice("/dev/ttyUSB0");

vector <Point> shapeNut(28);
    int YellowLowH = 20; // 25;
    int YellowHighH = 40; // 57;

    int YellowLowS = 150; // 115;
    int YellowHighS = 255; // 255;

    int YellowLowV = 140;// 132;
    int YellowHighV = 255; // 255;

    int SilverLowH = 0;// 51; //0
    int SilverHighH = 190; //179

    int SilverLowS = 0; //0
    int SilverHighS = 140;// 117; //121

    int SilverLowV = 121;//136; //121
    int SilverHighV = 255; //255


//--------------------------------------------functions--------------------------------------------------------
double axisProperties(vector<Point> contour)
{
	float majorAxis, minorAxis;
	Point center;
	if (contour.size()>=5)
	{	RotatedRect minEllipse = fitEllipse(contour);
		majorAxis = max(minEllipse.size.height, minEllipse.size.width);	
		minorAxis = min(minEllipse.size.height, minEllipse.size.width);	
		
		return (majorAxis/minorAxis);    }
}

//-------------------------------xxxxxxxxxxxxxxxxxxxxxxxx----------------------------------
void ObjClassifier();

//------------------------xxxxxxxxxxxxxxxxxxxxxxxxxxxxx--------------------------------------

static void mouseEvent (int evt, int x, int y, int flags, void *ptr)
{
	
	if(evt == CV_EVENT_LBUTTONDOWN)
        {  	Point*p = (Point*)ptr;
    		p->x = x;
    		p->y = y;
		cout << x << " , " << y <<  endl;

	}
	
	return ;

}

//------------------------xxxxxxxxxxxxxxxxxxxxxxxxxxxxx--------------------------------------

static void mainExit(GtkWidget* widget, gpointer data)
{
	g_print("Exiting Program.... \n");
    	exit (EXIT_SUCCESS);
}

//------------------------xxxxxxxxxxxxxxxxxxxxxxxxxxxxx--------------------------------------

static void Pickup(GtkWidget* widget, gpointer data)
{
	
	pickup = 5;
	//cout <<"PickUp Block Work!"<<"\t"<< "pickup = "<<pickup <<"\t"<< "\n" ;
}

//------------------------xxxxxxxxxxxxxxxxxxxxxxxxxxxxx--------------------------------------




static void AutoMode(GtkWidget* widget, gpointer data)
{
	if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget)))
	{	autoM = 5;
		
	}
	else
	{	autoM = 0;
			
	}
}


//------------------------xxxxxxxxxxxxxxxxxxxxxxxxxxxxx--------------------------------------

void UserInterface(int argc, char* argv[])   //GUI
{

	stringstream s;
    	string result;
    	char * data = (char*)'v';
	int flg = 0;
   	
	gtk_init(&argc,&argv);
	GtkWidget *window,*label,*button1,*tgbutton,*vbox,*hbox;
	window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	label = gtk_label_new("Manual Mode: \n Select object and press Pickup/Move");
	gtk_window_set_default_size(GTK_WINDOW(window),400,250);
	
	g_signal_connect(window,"delete-event",G_CALLBACK(mainExit),NULL);

	button1 = gtk_button_new_with_label("Pickup/Move");
	tgbutton = gtk_toggle_button_new_with_mnemonic("AUTO MODE");
	
	this_thread::sleep_for(chrono::milliseconds(500));

	hbox=gtk_hbox_new(0,0);
	vbox=gtk_vbox_new(0,0);

	gtk_box_pack_start(GTK_BOX(vbox),label,1,1,0);
	gtk_box_pack_start(GTK_BOX(vbox),hbox,1,1,0);

	gtk_box_pack_start(GTK_BOX(hbox),button1,1,1,0);
	gtk_box_pack_start(GTK_BOX(hbox),tgbutton,1,1,0);
	
	g_signal_connect(button1,"clicked",G_CALLBACK(Pickup),NULL);
	g_signal_connect(tgbutton,"toggled",G_CALLBACK(AutoMode),NULL);
 	gtk_container_add(GTK_CONTAINER(window),vbox);
	
	gtk_container_set_border_width (GTK_CONTAINER (window), 40);
	gtk_window_set_title(GTK_WINDOW(window), "ECASP Robotic Arm Control Panel");
    	gtk_widget_show_all(window);

	while(DeviceCheck){     //Serial Communication of data
		if (((object > 0)&&(pickup == 5))||(object>0 && autoM == 5))
		{
			cout << "Object" << object << endl;
			s.str("");
			s.clear();
			//Compensation for camera distortion
			OrtAng = cvRound(OrtAng);
			if (pkup.x < 0 && pkup.y < 180) //1
			s << cvRound(pkup.x*0.81+5) << "," << cvRound(pkup.y-5) << "," << object << "," << OrtAng << ",";
			else if (pkup.x > 0 && pkup.y < 180) //2
			s << cvRound(pkup.x*0.8) << "," << cvRound(pkup.y*1.05) << "," << object << "," << OrtAng << ",";
			else if (pkup.x > 0 && pkup.y > 180) //3
			s << cvRound(pkup.x*0.85) << "," << cvRound(pkup.y*1.04) << "," << object << "," << OrtAng << ",";
			else if (pkup.x < 0 && pkup.y > 180) //4
			s << cvRound(pkup.x*0.90) << "," << cvRound(pkup.y) << "," << object << "," << OrtAng << ",";
			result = s.str();
			data = (char*)result.c_str();
			printf("[%s]\n", data);
			int l = 0;
			if(autoM == 5 && flg == 0 )
			{	gtk_label_set_text(GTK_LABEL(label),"Auto Mode Enabled");
				flg = 5;
			}
				
			while (data[l] != '\0')
			{
				comm.send_data(data[l]);
				l++;
			}
		
			object = 0;
			pickup = 0;
			this_thread::sleep_for(chrono::seconds(3));
		}
		if(autoM == 0 && flg == 5)
		{
			gtk_label_set_text(GTK_LABEL(label),"Manual Mode: \n Select object and press Pickup");
			flg = 0;
		}

	}	
	return ;	
}	



int main(int argc, char* argv[])
{

    	
	//----------------Shape definitions---------------------------------
    shapeBolt2[0] = Point(274,138);     shapeBolt2[1] = Point(274,140);	
    shapeBolt2[2] = Point(273,141);     shapeBolt2[3] = Point(273,145);	
    shapeBolt2[4] = Point(274,146);     shapeBolt2[5] = Point(274, 169);	
    shapeBolt2[6] = Point(273, 170);     shapeBolt2[7] = Point(273, 179);	
    shapeBolt2[8] = Point(272, 180);     shapeBolt2[9] = Point(272, 183);	
    shapeBolt2[10] = Point(273, 184);     shapeBolt2[11] = Point(273, 185);	
    shapeBolt2[12] = Point(274, 186);     shapeBolt2[13] = Point(279, 186);	
    shapeBolt2[14] = Point(280, 185);    shapeBolt2[15] = Point(280, 180);	
    shapeBolt2[16] = Point(281, 179);    shapeBolt2[17] = Point(281, 165);	
    shapeBolt2[18] = Point(282, 164);    shapeBolt2[19] = Point(282, 149);	
    shapeBolt2[20] = Point(283, 148);    shapeBolt2[21] = Point(283, 147);	
    shapeBolt2[22] = Point(285, 145);    shapeBolt2[23] = Point(285, 139);	
    shapeBolt2[24] = Point(284, 138);    	
	    
    shapeBolt1[0] = Point(247, 139);     shapeBolt1[1] = Point(246, 140);	
    shapeBolt1[2] = Point(243, 140);     shapeBolt1[3] = Point(243, 141);	
    shapeBolt1[4] = Point(242, 142);     shapeBolt1[5] = Point(242, 145);	
    shapeBolt1[6] = Point(243, 146);     shapeBolt1[7] = Point(244, 146);	
    shapeBolt1[8] = Point(245, 147);     shapeBolt1[9] = Point(245, 151);	
    shapeBolt1[10] = Point(246, 152);     shapeBolt1[11] = Point(246, 173);	
    shapeBolt1[12] = Point(245, 174);     shapeBolt1[13] = Point(245, 182);	
    shapeBolt1[14] = Point(246, 183);    shapeBolt1[15] = Point(246, 184);	
    shapeBolt1[16] = Point(247, 185);    shapeBolt1[17] = Point(247, 186);	
    shapeBolt1[18] = Point(248, 187);    shapeBolt1[19] = Point(252, 187);	
    shapeBolt1[20] = Point(253, 186);    shapeBolt1[21] = Point(253, 185);	
    shapeBolt1[22] = Point(255, 183);    shapeBolt1[23] = Point(255, 166);	
    shapeBolt1[24] = Point(256, 165);    shapeBolt1[25] = Point(256, 157);	
    shapeBolt1[26] = Point(257, 156);    shapeBolt1[27] = Point(257, 148);	
    shapeBolt1[28] = Point(258, 147);    shapeBolt1[29] = Point(260, 147);	
    shapeBolt1[30] = Point(261, 146);    shapeBolt1[31] = Point(261, 142);	
    shapeBolt1[32] = Point(260, 141);    shapeBolt1[33] = Point(260, 140);	
    shapeBolt1[34] = Point(257, 140);    shapeBolt1[35] = Point(256, 139);	

    shapeNut[0] = Point(247,119);
    shapeNut[1] = Point(245,121);
    shapeNut[2] = Point(245,122);
    shapeNut[3] = Point(244,123);
    shapeNut[4] = Point(244,124);
    shapeNut[5] = Point(243,125);
    shapeNut[6] = Point(243,131);
    shapeNut[7] = Point(244,131);
    shapeNut[8] = Point(245,132);
    shapeNut[9] = Point(245,134);
    shapeNut[10] = Point(246,133);
    shapeNut[11] = Point(248,135);
    shapeNut[12] = Point(249,135);
    shapeNut[13] = Point(250,136);
    shapeNut[14] = Point(251,135);
    shapeNut[15] = Point(252,135);
    shapeNut[16] = Point(254,133);
    shapeNut[17] = Point(254,131);
    shapeNut[18] = Point(251,128);
    shapeNut[19] = Point(252,127);
    shapeNut[20] = Point(253,127);
    shapeNut[21] = Point(254,126);
    shapeNut[22] = Point(255,126);
    shapeNut[23] = Point(257,124);
    shapeNut[24] = Point(257,123);
    shapeNut[25] = Point(258,122);
    shapeNut[26] = Point(258,121);
    shapeNut[27] = Point(256,119); 
   
    gtk_disable_setlocale();

    namedWindow("Live image"); 
    setMouseCallback( "Live image", mouseEvent, &p); 
    //-----------------------Set Camera Properties------------------------------------------------------------------
    cap.open(0);
    cap.set(CV_CAP_PROP_BRIGHTNESS, -15);
    cap.set(CV_CAP_PROP_SATURATION, 200);
    cap.set(CV_CAP_PROP_CONTRAST, 10);

    if(!cap.isOpened())  
       {
        cout << "Error opening camera!";
        return -1;
       }

    //-------------------Camera Calibration Matrices-----------------------------------------------------------------------------------------------------------
    

    cameraMatrix.at<double>(0,0) = 6.0486080554129342e+002;   cameraMatrix.at<double>(0,1) = 0;     cameraMatrix.at<double>(0,2) = 3.1950000000000000e+002;
    cameraMatrix.at<double>(1,0) = 0;   cameraMatrix.at<double>(1,1) = 6.0486080554129342e+002;     cameraMatrix.at<double>(1,2) = 2.3950000000000000e+002;
    cameraMatrix.at<double>(2,0) = 0;   cameraMatrix.at<double>(2,1) = 0;     cameraMatrix.at<double>(2,2) = 1;
        
    
    distCoeffs.at<double>(0,0) = 8.2620139698452666e-002; distCoeffs.at<double>(1,0) = -2.7675886003881384e-001; distCoeffs.at<double>(2,0) = 0; 
    distCoeffs.at<double>(3,0) = 0; distCoeffs.at<double>(4,0) = 5.9528994991108919e-001;

    //--------------Multi-threading----------------------------------------------------------------------------------------------------------------------------
    thread first(UserInterface,argc,argv);
    thread second(ObjClassifier);

    first.join();
    second.join();
    
   
    
 
    
  return 0;
}


////--------------------------------------------------------------------------------------------------------------------------------/////

void ObjClassifier()    \\Image processing------------
{

Mat imgc, frame,framed; 
    
Mat yellow, imgHsv, silver, imgYellow, imgSilver ;	
int ddepth = CV_16S; 
double theta1, theta2, angle;
Point2d aux, auxtemp,aux1,aux2, aux1temp, aux2temp;
vector< vector<Point> > contours0, contours, contours1, contours2;
vector <Vec4i> hierarchy, hierarchy1, hierarchy2; 
vector<cv::Mat> values;
float RWx, RWy;
int loopCount = 0;
 
while(DeviceCheck)  
    {
        Mat imgGray,edges,imgfilt, imgThresh;
        cap >> framed; 
	int rcol = 640; int rlin = 480;
	int x1 = 60; int y1 = 0;
        int Bolts = 0, Nuts = 0;
	Rect roi(x1,y1,rcol - 150,rlin - 200);
	Point2d pt1 = Point2d(226,0); Point2d pt2 = Point2d(226,280);	
	Point2d pt3 = Point2d(0,3); Point2d pt4 = Point2d(490,3);
	
	
        int ddepth = CV_16S;

        int delta = 0;
        int scale = 1;
	undistort(framed,frame,cameraMatrix,distCoeffs,cameraMatrix);	
	
	//--------Edge Detection-------------------------------------------------
	frame = frame(roi);
	flip(frame,frame,-1);
        cvtColor(frame, imgGray, CV_BGR2GRAY);
        GaussianBlur(imgGray, imgGray, Size(3,3), 0, 0);

	Mat gdx,gdy,abs_gdx,abs_gdy;
	Sobel(imgGray, gdx, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(gdx,abs_gdx);

	Sobel(imgGray, gdy, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(gdy,abs_gdy);

	addWeighted(abs_gdx,0.5,abs_gdy,0.5, 0, edges);
	threshold(edges,edges,50,255,1);
	bitwise_not(edges,edges);

       	
        //namedWindow("Edges", WINDOW_NORMAL);
        //imshow("Edges", edges);
	
	//-------------Morphological Operations------------------------------------------
	Mat element = getStructuringElement(MORPH_RECT, Size(3,3), Point(-1,-1));
	morphologyEx(edges,imgfilt,MORPH_OPEN,element);
	morphologyEx(imgfilt,imgfilt,MORPH_CLOSE,element,Point(-1,-1),3);
	dilate(imgfilt,imgfilt, getStructuringElement(MORPH_ELLIPSE, Size(1,1)));
	


	//-------------------Contour Operations-------------------------------------------------
	findContours(imgfilt.clone(), contours0, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	for (int i = 0; i < contours0.size(); i++){
		double area  = contourArea(contours0[i]);
		
		if (area > 0 && area <= 200)
			drawContours(imgfilt, contours0, i, CV_RGB(0,0,0), -1);
	}
	threshold(imgfilt,imgc, 0, 255, 0); threshold(imgfilt,imgThresh, 0, 255, 0);

	
 

	blur(frame, frame, Size(3, 3), Point(-1, -1));
        
	
	//----------------------Classifier---------------------------------------------------------------

	//OTHER OBJECTS<<<<<<<<<<
        findContours(imgc.clone(), contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	vector<Point2d> massCenter(contours.size());
	vector<Moments> moment(contours.size());
		
	for (int i = 0; i < contours.size(); i++)
	{
		moment[i] = moments(contours[i], false);
		massCenter[i] = Point2d(moment[i].m10 / moment[i].m00, moment[i].m01 / moment[i].m00);
		
	}

	if (!contours.empty() && !hierarchy.empty())
	{

		int idx = 0;
		for (; idx >= 0; idx = hierarchy[idx][0])
		{
			
			double compareBolt1 = matchShapes(contours[idx], shapeBolt1, CV_CONTOURS_MATCH_I3, 0.0);
			double compareBolt2 = matchShapes(contours[idx], shapeBolt2, CV_CONTOURS_MATCH_I3, 0.0);
			double compareNut = matchShapes(contours[idx], shapeNut, CV_CONTOURS_MATCH_I3, 0.0);
			Point2f circleCenter;
			float circleRadius;
			minEnclosingCircle(contours[idx], circleCenter, circleRadius);
			//cout << compareBolt1 << ", " << compareBolt2 << ", " << compareNut << "\n";
			aux.x = 0;
			aux.y = 0;
			aux.x = (-1 * (massCenter[idx].x - 222));
			aux.y = ((massCenter[idx].y) + 40);

			double check = pointPolygonTest(contours[idx], p,true);    //-------Check if point is in contour------------
    			if(check>=0)  //--------Real World labeling-----------------
			{	
				RWx = massCenter[idx].x*0.0977505112474437 + massCenter[idx].y*0 + 0;
				RWy = massCenter[idx].x*0.0001992745510725 + massCenter[idx].y*0.09744525547445255 - 0.09744525547445255;
				string RWxy1 = "[" + to_string(RWx) + ", " + to_string(RWy) + "]";
				putText(frame, RWxy1, Point2d(massCenter[idx].x,massCenter[idx].y + 16), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(100,100,250,255), 2);	
		 	}
			else 
			{	object = 0;	}


  			if ((compareNut > 0.55 && compareBolt1 > 0.5 ) && (arcLength(contours[idx], true) >= 190) && axisProperties(contours[idx]) > 2)
			{

				string tests = to_string(axisProperties(contours[idx]));

				putText(frame, "Other", massCenter[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 255), 2);
				if (contours[idx].size() >= 5)
				{
					RotatedRect rec = fitEllipse(contours[idx]);
					theta1 = atan(aux.y / aux.x) * 180 / PI;
					theta2 = (rec.angle);
					angle = theta1 - (90 - theta2);
					if(check>=0||(autoM==5 && object == 0 && (loopCount % 6 == 0)))
					{
						pkup.x = aux.x; pkup.y = aux.y;
						OrtAng = angle;
						object = 5;
						
					}
				}


			}
			string xy = "[" + to_string(cvRound(aux.x)) + ", " + to_string(cvRound(aux.y)) + "]";
			if ((PI*circleRadius*circleRadius > 3000 && axisProperties(contours[idx]) < 5.0)||(PI*circleRadius*circleRadius > 940 && axisProperties(contours[idx]) < 2.5))
			{
				putText(frame, "Overlapped" + xy, massCenter[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(100,250,100,255), 2);
				drawContours(imgThresh, contours, idx, CV_RGB(0,0,0), -1);
				if(check>=0||(autoM==5 && object == 0 && (loopCount % 6 == 0)))
				{
					pkup.x = aux.x; pkup.y = aux.y;
					OrtAng = 0;
					object = 6;
					
				}
			}
			
		}
		
	}
	

        //--------------------Color Threshold ----------------------------------------------------------------
	
	cvtColor(frame, imgHsv, COLOR_BGR2HSV);
	split(imgHsv, values);

	equalizeHist(values[1], values[1]);

	merge(values, imgHsv);
	//imshow("hsv", imgHsv);
	
	
	//Yellow
	inRange(imgHsv, Scalar(YellowLowH, YellowLowS, YellowLowV), Scalar(YellowHighH, YellowHighS, YellowHighV), yellow);
	
	dilate(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));
	dilate(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
	
	findContours(yellow.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	for (int i = 0; i < contours.size(); i++)
	{
		// Calculate contour area
		double area = contourArea(contours[i]);

		// Remove small objects by drawing the contour with black color
		if (area > 0 && area <= 190)
			drawContours(yellow, contours, i, CV_RGB(0, 0, 0), -1);
	}
	threshold(yellow, imgYellow, 0 , 255, 0);
	bitwise_and(imgYellow,imgThresh,imgYellow);	
	//imshow("Image Yellow", imgYellow);
	

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
	bitwise_and(imgSilver,imgThresh,imgSilver);
	//imshow("Silver", imgSilver);

       //-----------------------------------------------------------------------------------------------------------------------------------
	//>>>>>YELLOW OBJECTS<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	findContours(imgYellow.clone(), contours2, hierarchy2, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	
	vector<Point2d> massCenter2(contours2.size());
	vector<Moments> moment2(contours2.size());
	
	for (int i = 0; i < contours2.size(); i++)
	{
		moment2[i] = moments(contours2[i], false);
		massCenter2[i] = Point2d(moment2[i].m10/moment2[i].m00, moment2[i].m01/moment2[i].m00);
	}


	if (!contours2.empty() && !hierarchy2.empty())
	{
		aux2 = Point2d(0,0); aux2temp = Point2d(0,0);

		int idx = 0;
		for(; idx >= 0 ; idx = hierarchy2[idx][0])
		{

			
			aux2.x = 0; aux2.y = 0;
			
			aux2.x = (-1 * (massCenter2[idx].x - 250));
			aux2.y = ((massCenter2[idx].y - 5 ));
			
						
			auxtemp.x = (-1 * (massCenter2[idx].x - 226));
			auxtemp.y = ((massCenter2[idx].y + 40 ));
			
			  
			aux2.x = aux2.x*(46.8/490); aux2.y = aux2.y*(26.7/280);

			string xy2 = "[" + to_string(cvRound(aux2.x)) + ", " + to_string(cvRound(aux2.y)) + "]";
			
			
			Point2f circleCenter2;
			float circleRadius2;
			minEnclosingCircle(contours2[idx], circleCenter2, circleRadius2);

			double check = pointPolygonTest(contours2[idx], p,true);

			
			if ((axisProperties(contours2[idx]) >= 2.0) && (axisProperties(contours2[idx]) < 10) && (arcLength(contours2[idx], true) >= 95) && (arcLength(contours2[idx], true) <= 190))	
			{
				RotatedRect rec = fitEllipse(contours2[idx]);
				theta1 = atan(auxtemp.y/auxtemp.x) * 180/ PI;
				theta2 = (rec.angle);
				angle = theta1 - (90 - theta2);
				
				string xy2 = "[" + to_string(cvRound(aux2.x)) + ", " + to_string(cvRound(aux2.y)) + ", " + to_string(cvRound(angle)) + "deg]";

				string test = to_string(cvRound(theta1)) + ", " + to_string(cvRound(theta2)) + ", " + to_string(cvRound(angle));
				putText(frame, "Yellow Bolt" + xy2, massCenter2[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(100,250,100,255), 2);
				if(check>=0||(autoM==5 && object == 0 && (loopCount % 6 == 0)))
				{
					pkup.x = auxtemp.x; pkup.y = auxtemp.y;
					OrtAng = angle;
					object = 1;
					
				}
			}
			else if ((axisProperties(contours2[idx]) < 1.3) && (arcLength(contours2[idx], true) <290))
			{
				putText(frame, "Yellow Nut" + xy2, massCenter2[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(100,250,100,255), 2);
				
				angle = 0;
				if(check>=0||(autoM==5 && object == 0 && (loopCount % 6 == 0)))
				{
					pkup.x = auxtemp.x; pkup.y = auxtemp.y;
					OrtAng = angle;
					object = 2;
					
				}
			}
	 		else
			{
				putText(frame, "Unknown Yellow Object" + xy2, massCenter2[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(100,250,100,255), 2);
				//cout << PI*circleRadius2*circleRadius2 << "\n" << axisProperties(contours2[idx]) << "\n" << arcLength(contours2[idx],true) << endl ;
				if(check>=0||(autoM==5 && object == 0 && (loopCount % 6 == 0)))
				{
					pkup.x = auxtemp.x; pkup.y = auxtemp.y;
					OrtAng = 0;
					object = 6;
					
				}
			}
		}
	}
	
	//>>>>>>SILVER OBJECTS<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	findContours(imgSilver.clone(), contours1, hierarchy1, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	
	vector<Point2d> massCenter1(contours1.size());
	vector<Moments> moment1(contours1.size());
	
	for (int i = 0; i < contours1.size(); i++)
	{
		moment1[i] = moments(contours1[i], false);
		massCenter1[i] = Point2d(moment1[i].m10/moment1[i].m00, moment1[i].m01/moment1[i].m00);
	}


	if (!contours1.empty() && !hierarchy1.empty())
	{
		aux1 = Point2d(0,0); aux1temp = Point2d(0,0);

		int idx = 0;
		for(; idx >= 0 ; idx = hierarchy1[idx][0])
		{

			
			aux1.x = 0; aux1.y = 0;
			
			aux1.x = (-1 * (massCenter1[idx].x - 250));
			aux1.y = ((massCenter1[idx].y - 5 ));
			
			
			
			aux1temp.x = (-1 * (massCenter1[idx].x - 226));
			aux1temp.y = ((massCenter1[idx].y + 40 ));
			
			  
			aux1.x = aux1.x*(46.8/490); aux1.y = aux1.y*(26.7/280);

			string xy1 = "[" + to_string(cvRound(aux1.x)) + ", " + to_string(cvRound(aux1.y)) + "]";
			
			
			Point2f circleCenter1;
			float circleRadius1;
			minEnclosingCircle(contours1[idx], circleCenter1, circleRadius1);

			double check = pointPolygonTest(contours1[idx], p,true);
			
			if ((axisProperties(contours1[idx]) >= 2.50) && (axisProperties(contours1[idx]) < 10) && (arcLength(contours1[idx], true) >= 100) && (arcLength(contours1[idx], true) <= 190))	
			{
				RotatedRect rec = fitEllipse(contours1[idx]);
				theta1 = atan(auxtemp.y/auxtemp.x) * 180/ PI;
				theta2 = (rec.angle);
				angle = theta1 - (90 - theta2);
				

				string xy1 = "[" + to_string(cvRound(aux1.x)) + ", " + to_string(cvRound(aux1.y)) + ", " + to_string(cvRound(angle)) + "deg]";

				
				putText(frame, "Silver Bolt" + xy1, massCenter1[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(100,250,100,255), 2);
				if(check>=0||(autoM==5 && object == 0 && (loopCount % 6 == 0)))
				{
					pkup.x = aux1temp.x; pkup.y = aux1temp.y;
					OrtAng = angle;
					object = 3;
					
				}
				
			}
			else if ((axisProperties(contours1[idx]) < 1.3) && (arcLength(contours1[idx], true) <290))
			{
				putText(frame, "Silver Nut" + xy1, massCenter1[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(100,250,100,255), 2);
				angle = 0;
				if(check>=0||(autoM==5 && object == 0 && (loopCount % 6 == 0)))
				{
					pkup.x = aux1temp.x; pkup.y = aux1temp.y;
					OrtAng = angle;
					object = 4;
					
				}
			}
	 		else
			{
				putText(frame, "Unknown Silver Object" + xy1, massCenter1[idx], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(100,250,100,255), 2);
				//cout << PI*circleRadius1*circleRadius1 << "\n" << axisProperties(contours1[idx]) << "\n" << arcLength(contours1[idx],true) << endl ;
				if(check>=0||(autoM==5 && object == 0 && (loopCount % 6 == 0)))
				{
					pkup.x = aux1temp.x; pkup.y = aux1temp.y;
					OrtAng = 0;
					object = 6;
					
				}
								
			}
		}
	}
    	cv::line(frame, pt1, pt2, Scalar(250,250,250));
    	cv::line(frame, pt3, pt4, Scalar(250,250,250));
    	namedWindow("ALL",0);
    	imshow("ALL", imgc );
    	imshow("Live image", frame);
    	loopCount++;
    	if(waitKey(30) == 27)
    	{	cv::destroyAllWindows(); 
	   	cout << "Exiting Program ..." << endl ;
 	    	exit(EXIT_SUCCESS);                    }
  }

return ;
}



///----------------------------------MEASURES FOR REAL WORLD COORDINATE-----------------------------------------------------------------------------------------------------//
/*
Origin at left top corner (near Robo Arm) ----> Coordinates with respect to bottom left corner (away from Robo Arm) = [6.5cm , 26.8cm]
Rough sketch>    --------0rigin---------------Robo Arm----------------------B
		 |         |                                                |
	26.8cm ->|	   |                                                | 
		 |         |                                                |
		 |         |                                                | 
		 |--6.5cm--C------------------------------------------------D	

O ---> [0,1] pixels =  [0,0] cm
B ---> [489,0] pixels =  [47.8 , 0] cm
C ---> [0,276] pix  =  [0,26.3] cm
D ---> [489,274] pix =  [47.8, 26.7] cm

[ X ; Y ] = [ a00 a01 ; a10 a11] * [ u ; v ] + [tx ; ty] 

*/








