
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string> 
#include <sstream>
#include <stdio.h>
#include <opencv2/cudacodec.hpp>
#include "lsf.h"
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */
#define ARRAY_SIZE 6
// Select Video Source
// The MP4 demo uses a ROI for better tracking of the moving object
#define TEST_LIVE_VIDEO


using namespace cv;
using namespace std;

int lambda1X = 447;
int lambda1Y = 249;
int lambda2X = 1029;
int lambda2Y = 216;
int lambda3X = 1079;
int lambda3Y = 548;
int lambda4X = 448;
int lambda4Y = 580;


int topH_MIN = 85;
int topH_MAX = 256;
int topS_MIN = 150;
int topS_MAX = 256;
int topV_MIN = 157;
int topV_MAX = 256;

int botH_MIN = 32;
int botH_MAX = 256;
int botS_MIN = 0;
int botS_MAX = 86;
int botV_MIN = 102;
int botV_MAX = 181;

int gameH_MIN = 0;
int gameH_MAX = 256;
int gameS_MIN = 0;
int gameS_MAX = 256;
int gameV_MIN = 132;
int gameV_MAX = 256;

int theObjectArray[2][ARRAY_SIZE] = {};
int theObject[2]= {};
int theObject_prev[2] = {};
int array_index = 0;
int roi_xmin = 455;
int roi_width = 305;
int roi_ymin = 95;
int roi_height = 525; 

int top_roi_xmin = 455;
int top_roi_width = 305;
int top_roi_ymin = 60;
int top_roi_height = 30; 

int bot_roi_xmin = 455;
int bot_roi_width = 300;
int bot_roi_ymin = 620;
int bot_roi_height = 30; 

std::deque<cv::Point3i> corners;

int n_rebounds;
int first_y_rebound;
bool y_dir;
const string trackbarWindowName_bot = "TrackbarBottomPaddle";
const string trackbarWindowName_top = "TrackbarTopPaddle";
const string trackbarWindowName_game = "TrackbarGame";
const string trackbarWindowName_coords = "TrackbarsCoordinates";

int returnLoc;
int currentLoc;
bool disappeared = 0;

cv::Point2i collision0;
cv::Point2i collision1;
cv::Rect roi(roi_xmin, roi_ymin, roi_width, roi_height);
cv::Rect top_roi(top_roi_xmin, top_roi_ymin, top_roi_width, top_roi_height);
cv::Rect bot_roi(bot_roi_xmin, bot_roi_ymin, bot_roi_width, bot_roi_height);
lsf my_lsf(roi, ARRAY_SIZE);
//my_lsf.addPoint(Point2i(25,30));
//bounding rectangle of the object, we will use the center of this as its position.
cv::Rect objectBoundingRectangle = cv::Rect(0,0,0,0);


char write_buffer[1];
char dur_buffer[10];
char read_buffer[1];
int  bytes_written;  
int  bytes_read; 
struct termios options;           // Terminal options
int fd;                           // File descriptor for the port
	
int difference;
int proportional;


void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed
}

void findCorners(cv::Mat src_gray, cv::Mat &cameraFeed){
	cv::Mat dst;
	cv::Mat dst_norm;
	dst = cv::Mat::zeros(src_gray.size(), CV_32FC1);	

	int blockSize = 2;
	int apertureSize = 3;
	double k = .08;
	int thresh = 200;

	corners.clear();

	cv::cornerHarris(src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT);
	cv::normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
	
	for(int j = 0; j < dst_norm.rows; j++){
		for(int i = 0; i < dst_norm.cols; i++){
			if((int) dst_norm.at<float>(j,i) > thresh){
				corners.push_front(Point3i(i, j, dst_norm.at<float>(j,i)));
				//cv::circle(cameraFeed, Point3i(i,j,), 5, Scalar(0,255,0), 2, 8, 0);
			}
		}
	}
	
	for(int i = 0; i < corners.size(); i++){
		cout << "Corners are at:  " << corners.at(i) << endl;	
	}

}


void createTrackbars(){
	//create window for trackbars
    namedWindow(trackbarWindowName_bot,0);
	namedWindow(trackbarWindowName_top,0);
	namedWindow(trackbarWindowName_game,0);
	namedWindow(trackbarWindowName_coords,0);

	//create memory to store trackbar name on window
	char TrackbarName_bot[50];
	char TrackbarName_top[50];
	char TrackbarName_game[50];
	char TrackbarName_coords[50];

	sprintf( TrackbarName_coords, "lambda1X", lambda1X);
	sprintf( TrackbarName_coords, "lambda1Y", lambda1Y);
	sprintf( TrackbarName_coords, "lambda2X", lambda2X);
	sprintf( TrackbarName_coords, "lambda2Y", lambda2Y);
	sprintf( TrackbarName_coords, "lambda3X", lambda3X);
	sprintf( TrackbarName_coords, "lambda3Y", lambda3Y);
	sprintf( TrackbarName_coords, "lambda4X", lambda4X);
	sprintf( TrackbarName_coords, "lambda4Y", lambda4Y);

	sprintf( TrackbarName_bot, "H_MIN_BOT", botH_MIN);
	sprintf( TrackbarName_bot, "H_MAX_BOT", botH_MAX);
	sprintf( TrackbarName_bot, "S_MIN_BOT", botS_MIN);
	sprintf( TrackbarName_bot, "S_MAX_BOT", botS_MAX);
	sprintf( TrackbarName_bot, "V_MIN_BOT", botV_MIN);
	sprintf( TrackbarName_bot, "V_MAX_BOT", botV_MAX);

	sprintf( TrackbarName_top, "H_MIN_TOP", topH_MIN);
	sprintf( TrackbarName_top, "H_MAX_TOP", topH_MAX);
	sprintf( TrackbarName_top, "S_MIN_TOP", topS_MIN);
	sprintf( TrackbarName_top, "S_MAX_TOP", topS_MAX);
	sprintf( TrackbarName_top, "V_MIN_TOP", topV_MIN);
	sprintf( TrackbarName_top, "V_MAX_TOP", topV_MAX);

	sprintf( TrackbarName_top, "H_MIN_game", gameH_MIN);
	sprintf( TrackbarName_top, "H_MAX_game", gameH_MAX);
	sprintf( TrackbarName_top, "S_MIN_game", gameS_MIN);
	sprintf( TrackbarName_top, "S_MAX_game", gameS_MAX);
	sprintf( TrackbarName_top, "V_MIN_game", gameV_MIN);
	sprintf( TrackbarName_top, "V_MAX_game", gameV_MAX);


	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      

    createTrackbar( "lambda1X", trackbarWindowName_coords, &lambda1X, 1280, on_trackbar );
    createTrackbar( "lambda1Y", trackbarWindowName_coords, &lambda1Y, 720, on_trackbar );
    createTrackbar( "lambda2X", trackbarWindowName_coords, &lambda2X, 1280, on_trackbar );
    createTrackbar( "lambda2Y", trackbarWindowName_coords, &lambda2Y, 720, on_trackbar );
    createTrackbar( "lambda3X", trackbarWindowName_coords, &lambda3X, 1280, on_trackbar );
    createTrackbar( "lambda3Y", trackbarWindowName_coords, &lambda3Y, 720, on_trackbar );
    createTrackbar( "lambda4X", trackbarWindowName_coords, &lambda4X, 1280, on_trackbar );
    createTrackbar( "lambda4Y", trackbarWindowName_coords, &lambda4Y, 720, on_trackbar );

    createTrackbar( "H_MIN_bot", trackbarWindowName_bot, &botH_MIN, 256, on_trackbar );
    createTrackbar( "H_MAX_bot", trackbarWindowName_bot, &botH_MAX, 256, on_trackbar );
    createTrackbar( "S_MIN_bot", trackbarWindowName_bot, &botS_MIN, 256, on_trackbar );
    createTrackbar( "S_MAX_bot", trackbarWindowName_bot, &botS_MAX, 256, on_trackbar );
    createTrackbar( "V_MIN_bot", trackbarWindowName_bot, &botV_MIN, 256, on_trackbar );
    createTrackbar( "V_MAX_bot", trackbarWindowName_bot, &botV_MAX, 256, on_trackbar );

    createTrackbar( "H_MIN_top", trackbarWindowName_top, &topH_MIN, 256, on_trackbar );
    createTrackbar( "H_MAX_top", trackbarWindowName_top, &topH_MAX, 256, on_trackbar );
    createTrackbar( "S_MIN_top", trackbarWindowName_top, &topS_MIN, 256, on_trackbar );
    createTrackbar( "S_MAX_top", trackbarWindowName_top, &topS_MAX, 256, on_trackbar );
    createTrackbar( "V_MIN_top", trackbarWindowName_top, &topV_MIN, 256, on_trackbar );
    createTrackbar( "V_MAX_top", trackbarWindowName_top, &topV_MAX, 256, on_trackbar );

    createTrackbar( "H_MIN_game", trackbarWindowName_game, &gameH_MIN, 256, on_trackbar );
    createTrackbar( "H_MAX_game", trackbarWindowName_game, &gameH_MAX, 256, on_trackbar );
    createTrackbar( "S_MIN_game", trackbarWindowName_game, &gameS_MIN, 256, on_trackbar );
    createTrackbar( "S_MAX_game", trackbarWindowName_game, &gameS_MAX, 256, on_trackbar );
    createTrackbar( "V_MIN_game", trackbarWindowName_game, &gameV_MIN, 256, on_trackbar );
    createTrackbar( "V_MAX_game", trackbarWindowName_game, &gameV_MAX, 256, on_trackbar );
}




//-----------------------------------------------------------------------------------------------------------------
// int to string helper function
//-----------------------------------------------------------------------------------------------------------------
string intToString(int number){
 
    //this function has a number input and string output
    std::stringstream ss;
    ss << number;
    return ss.str();
}



//-----------------------------------------------------------------------------------------------------------------
// Search for Paddles
//--------------------------------------------------------------------------------------------------------------
void searchForPaddles(cv::Rect roi, cv:: Mat thresholdImage, cv::Mat &cameraFeed){

	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	cv::Rect Pad = cv::Rect(0,0,0,0);
	cv::Mat temp;
 
	thresholdImage.copyTo(temp);
	cv::Mat roi_temp = temp(roi);
	cv::findContours(roi_temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	if(contours.size()>0){
		vector< vector<Point> > largestContourVec;
		largestContourVec.push_back(contours.at(contours.size()-1));
		Pad = boundingRect(largestContourVec.at(0));	
	}

	line(cameraFeed, Point(roi.x, roi.y), Point(roi.x+roi.width, roi.y),Scalar(0,0,255),2);
	line(cameraFeed, Point(roi.x, roi.y), Point(roi.x, roi.y+roi.height),Scalar(0,0,255),2);
	line(cameraFeed, Point(roi.x+roi.width, roi.y), Point(roi.x+roi.width, roi.y+roi.height),Scalar(0,0,255),2);
	line(cameraFeed, Point(roi.x, roi.y+roi.height), Point(roi.x+roi.width, roi.y+roi.height),Scalar(0,0,255),2);

	Pad.x = Pad.x + roi.x;
	Pad.y = Pad.y + roi.y;

	currentLoc = Pad.x+10;

	if((Pad.width < 2) && (Pad.x == roi.x)){
		disappeared = 1;	
	}
	else
	disappeared = 0;


	if(contours.size()>0){
		line(cameraFeed, Point(Pad.x, Pad.y), Point(Pad.x+Pad.width, Pad.y),Scalar(0,0,255),2);
		line(cameraFeed, Point(Pad.x, Pad.y), Point(Pad.x, Pad.y+Pad.height),Scalar(0,0,255),2);
		line(cameraFeed, Point(Pad.x+Pad.width, Pad.y), Point(Pad.x+Pad.width, Pad.y+Pad.height),Scalar(0,0,255),2);
		line(cameraFeed, Point(Pad.x, Pad.y+Pad.height), Point(Pad.x+Pad.width, Pad.y+Pad.height),Scalar(0,0,255),2);
	}

}



//-----------------------------------------------------------------------------------------------------------------
// Search for Moving Object
//-----------------------------------------------------------------------------------------------------------------
void searchForMovement(cv::Mat thresholdImage, cv::Mat &cameraFeed){
    //notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
    //to take the values passed into the function and manipulate them, rather than just working with a copy.
    //eg. we draw to the cameraFeed to be displayed in the main() function.

    bool objectDetected = false;
    int xpos, ypos;
    float deltax = 0;
    float deltay = 0;
    float x_vel, y_vel;
	float x_avg = 0;
	float y_avg = 0;
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;

    cv::Mat temp;

    thresholdImage.copyTo(temp);

#ifdef TEST_LIVE_VIDEO
    cv::Mat roi_temp = temp(roi); 

    //find contours of filtered image using openCV findContours function
    cv::findContours(roi_temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours
    //Draw a rectangle around ROI
    rectangle(cameraFeed, Point(roi_xmin,roi_ymin), Point(roi_xmin+roi_width, roi_ymin+roi_height), Scalar(255,0,0), 3);

#else
    cv::Mat roi_temp = temp(roi); 

    //find contours of filtered image using openCV findContours function
    cv::findContours(roi_temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours
    //Draw a rectangle around ROI
    rectangle(cameraFeed, Point(roi_xmin,roi_ymin), Point(roi_xmin+roi_width, roi_ymin+roi_height), Scalar(255,0,0), 3);

#endif

    //if contours vector is not empty, we have found some objects
    if(contours.size()>0)
	objectDetected = true;
    else 
	objectDetected = false;
 


    if(objectDetected){

        //the largest contour is found at the end of the contours vector
        //we will simply assume that the biggest contour is the object we are looking for.
        vector< vector<Point> > largestContourVec;
        largestContourVec.push_back(contours.at(contours.size()-1));

        //make a bounding rectangle around the largest contour then find its centroid
        //this will be the object's final estimated position.
        objectBoundingRectangle = boundingRect(largestContourVec.at(0));

        xpos = objectBoundingRectangle.x+objectBoundingRectangle.width/2;
        ypos = objectBoundingRectangle.y+objectBoundingRectangle.height/2;

		circle(cameraFeed,Point(xpos,ypos),10,cv::Scalar(0,255,0),2);

		int monotonic;
		bool horizontal_wall = false;
		
		if(my_lsf.getSize() < ARRAY_SIZE) {
			y_dir = my_lsf.addPoint(Point2i(xpos,ypos));
			cout << "Added point (low array size)" << endl;
		}else{
			if(my_lsf.testMonotonic()){
				my_lsf.computeLSF();
				n_rebounds = my_lsf.computeRebounds(Point2i(xpos,ypos));
				
				first_y_rebound = my_lsf.getSecondToLastPoint();
				cout << "Problematic Number:  " << first_y_rebound << endl;
				first_y_rebound = my_lsf.getRebound(n_rebounds - 2).y;	//THIS IS THE PROBLEM MAYBE
				cout << "Problematic Number:  " << first_y_rebound << endl;
			}
			if(y_dir && ((ypos + roi_ymin) < first_y_rebound)){
				y_dir = my_lsf.addPoint(Point2i(xpos,ypos));
				cout << "All Collisions:   "  << endl;
				for(int i = 0; i < (n_rebounds); i++){
					cout << my_lsf.getRebound(i) << endl;
				}
			returnLoc = my_lsf.getRebound(0).x;
			for(int i = 0; i < (n_rebounds-1); i++){
				line(cameraFeed,my_lsf.getRebound(i), my_lsf.getRebound(i+1),Scalar(0,0,255),2);
			}
			}else{
				y_dir = my_lsf.addPoint(Point2i(xpos,ypos));
				my_lsf.trimPoints(ARRAY_SIZE);
			}
		}		
		
// && (((xpos-roi_xmin) > 20) || (((roi_xmin+roi_width)-xpos) > 20))
// 			cout << "YDIR, YPOS+ROIYMIN, REBOUND N-1:   "<< y_dir << "  " << (ypos + roi_ymin) << "  " << my_lsf.getRebound(n_rebounds-2).y << endl;
		theObject_prev[0] = theObject[0];
		theObject_prev[1] = theObject[1];
		theObject[0] = xpos;
		theObject[1] = ypos;
	
		deltax = theObject_prev[0] - theObject[0];
		deltay = theObject_prev[1] - theObject[1];


		// calculate the x and y components of velocity using 30fps (1/30 = .03333), multiply by .5 to scale.
		x_vel = (deltax/(0.03333))*.25;
		y_vel = (deltay/(0.03333))*.25;
    	}

    	//make some temp x and y variables so we dont have to type out so much

#ifdef TEST_LIVE_VIDEO

    int x = theObject[0];
    int y = theObject[1];

#else
    int x = theObject[0]+roi_xmin;
    int y = theObject[1]+roi_ymin;

#endif

}




void writeArduino(char command, int duration){

  memset(&write_buffer, 0, sizeof(write_buffer)); //Clear the Array
  memset(&dur_buffer, 0, sizeof(dur_buffer)); //Clear the Array
  memset(&read_buffer, 0, sizeof(read_buffer));

  bytes_written  =  0;

  if(command == 'R'){
	strcpy(write_buffer, "R");
  }
  else if(command == 'L'){
	strcpy(write_buffer, "L");
  }
  else if(command == 'S'){
	strcpy(write_buffer, "S");
  }

  sprintf(dur_buffer, "%d ", duration);
  
  //printf("DurBuffer %s\n", dur_buffer);
  //printf("writeBuffer %s\n", write_buffer);

  strcat(write_buffer, dur_buffer);
                                                        
  bytes_written = write(fd, &write_buffer, strlen(write_buffer));
  if (bytes_written == -1)
	printf("W Error=%d\n", errno);
  else
    printf("Wrote=%s\n", write_buffer);
  // Check response
  while(read(fd, &read_buffer, 1) != 1) {} 
  printf("Read=%c\n", read_buffer[0]);
}





//-----------------------------------------------------------------------------------------------------------------
// MAIN 
//-----------------------------------------------------------------------------------------------------------------
int main() {



//----------------------------------------
// Serial Communication Stuff.
//---------------------------------------
  fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);   // Open tty device for RD and WR
  usleep(2000000);

  if(fd == 1) {
     printf("\n  Error! in Opening ttyUSB0\n");
  }
  else
     printf("\n  ttyUSB0 Opened Successfully\n");

    tcgetattr(fd, &options);               // Get the current options for the port
    cfsetispeed(&options, B115200);        // Set the baud rates to 115200          
    cfsetospeed(&options, B115200);                   
    options.c_cflag |= (CLOCAL | CREAD);   // Enable the receiver and set local mode           
    options.c_cflag &= ~PARENB;            // No parity                 
    options.c_cflag &= ~CSTOPB;            // 1 stop bit                  
    options.c_cflag &= ~CSIZE;             // Mask data size         
    options.c_cflag |= CS8;                // 8 bits
    options.c_cflag &= ~CRTSCTS;           // Disable hardware flow control    

// Enable data to be processed as raw input
    options.c_lflag &= ~(ICANON | ECHO | ISIG);
     
    tcsetattr(fd, TCSANOW, &options);      // Apply options immediately
    fcntl(fd, F_SETFL, FNDELAY);    




//----------------------------------------
// Video/Matrix/CV stuff
//---------------------------------------
    // OpenCV frame matrices
    cv::Mat frame0, frame1, frame0_warped, frame1_warped, frame0_hsv, frame1_hsv, frame0_hsv_warped, frame1_hsv_warped, result, topThreshold, botThreshold, gameThreshold;

    cv::cuda::GpuMat gpu_frame0, gpu_frame0_warped, gpu_frame1, gpu_frame1_warped, gpu_grayImage0, 	gpu_grayImage1, gpu_differenceImage, gpu_thresholdImage, gpu_frame0_hsv_warped, gpu_frame1_hsv_warped, gpu_frame0_hsv, gpu_frame1_hsv;
			
    int toggle, frame_count;

#ifdef TEST_LIVE_VIDEO

    // Camera video pipeline
    std::string pipeline = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

#else

    // MP4 file pipeline
    std::string pipeline = "filesrc location=/home/nvidia/Downloads/pong_video.mp4 ! qtdemux name=demux ! h264parse ! omxh264dec ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

#endif

    std::cout << "Using pipeline: " << pipeline << std::endl;
 
    // Create OpenCV capture object, ensure it works.
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        std::cout << "Connection failed" << std::endl;
        return -1;
    }

	createTrackbars();

    // Transformation Matrix Variables
	// Input and output quadrilaterals    
    Point2f inputQuad[4];
	Point2f outputQuad[4];
	
	//Lambda Matrix for transformation
	cv::Mat lambda(2, 4, CV_32FC1);

	// Predefined input quadrilateral (locations of corners) (Skewed Rectangle)
	inputQuad[3] = Point2f(520,80);
	inputQuad[0] = Point2f(880,77);
	inputQuad[1] = Point2f(923,672);
	inputQuad[2] = Point2f(472,655);

	// Where to remap the output quadrilateral (Fixed Rectangle)
	outputQuad[0] = Point2f(437,0);
	outputQuad[1] = Point2f(842,0);
	outputQuad[2] = Point2f(842,719);
	outputQuad[3] = Point2f(437,719);

	// Get Lambda from the image transform function:
	lambda = cv::getPerspectiveTransform(inputQuad,outputQuad);

    // Capture the first frame with GStreamer
    cap >> frame0;
	
	//Upload to GPU
    gpu_frame0.upload(frame0);

	// Warp Perspective
	cv::cuda::warpPerspective(gpu_frame0, gpu_frame0_warped, lambda, gpu_frame0.size());
	gpu_frame0_warped.download(frame0_warped);	
	
    // Convert the frames to gray scale (monochrome)
    cv::cuda::cvtColor(gpu_frame0_warped,gpu_grayImage0,cv::COLOR_BGR2GRAY);

	// Convert the frames to HSV
	cv::cuda::cvtColor(gpu_frame0_warped,gpu_frame0_hsv_warped,COLOR_BGR2HSV);
	cv::cuda::cvtColor(gpu_frame0,gpu_frame0_hsv,COLOR_BGR2HSV);
	gpu_frame0_hsv.download(frame0_hsv);
	gpu_frame0_hsv_warped.download(frame0_hsv_warped);
	inRange(frame0_hsv_warped,Scalar(topH_MIN,topS_MIN,topV_MIN),Scalar(topH_MAX,topS_MAX,topV_MAX),topThreshold);
	inRange(frame0_hsv_warped,Scalar(botH_MIN,botS_MIN,botV_MIN),Scalar(botH_MAX,botS_MAX,botV_MAX),botThreshold);
	inRange(frame0_hsv,Scalar(gameH_MIN,gameS_MIN,gameV_MIN),Scalar(gameH_MAX,gameS_MAX,gameV_MAX),gameThreshold);
	
    // Initialize 
    toggle = 0;
    frame_count = 0;

//frame_count < 100
//    while (frame_count < 10000) {
while(1){





        if (toggle == 0) {
           	// Get a new frame from file and upload to GPU
           	cap >> frame1;
	   		gpu_frame1.upload(frame1);
			
			// Warp Perspective in GPU
			cv::cuda::warpPerspective(gpu_frame1, gpu_frame1_warped, lambda, gpu_frame1.size());
			gpu_frame1_warped.download(frame1_warped);	
			
           	// Convert the frames to gray scale (monochrome)    
           	cv::cuda::cvtColor(gpu_frame1_warped,gpu_grayImage1,cv::COLOR_BGR2GRAY);

			// Convert the frames to HSV
			cv::cuda::cvtColor(gpu_frame1_warped,gpu_frame1_hsv_warped,COLOR_BGR2HSV);
			gpu_frame1_hsv_warped.download(frame1_hsv_warped);
			cv::cuda::cvtColor(gpu_frame1,gpu_frame1_hsv,COLOR_BGR2HSV);
			gpu_frame1_hsv.download(frame1_hsv);
			inRange(frame1_hsv_warped,Scalar(topH_MIN,topS_MIN,topV_MIN),Scalar(topH_MAX,topS_MAX,topV_MAX),topThreshold);
			inRange(frame1_hsv_warped,Scalar(botH_MIN,botS_MIN,botV_MIN),Scalar(botH_MAX,botS_MAX,botV_MAX),botThreshold);
			inRange(frame1_hsv,Scalar(gameH_MIN,gameS_MIN,gameV_MIN),Scalar(gameH_MAX,gameS_MAX,gameV_MAX), gameThreshold);
		
          
			 
			toggle = 1;
        } 
        else { // Same as above, for frame0
			cap >> frame0;
           	gpu_frame0.upload(frame0);
			cv::cuda::warpPerspective(gpu_frame0, gpu_frame0_warped, lambda, gpu_frame0.size());
			gpu_frame0_warped.download(frame0_warped);
			
			// Convert to Gray scale	
           	cv::cuda::cvtColor(gpu_frame0_warped,gpu_grayImage0,cv::COLOR_BGR2GRAY);

			// Convert to HSV
			cv::cuda::cvtColor(gpu_frame0_warped,gpu_frame0_hsv_warped,COLOR_BGR2HSV);
			gpu_frame0_hsv_warped.download(frame0_hsv_warped);
			inRange(frame0_hsv_warped,Scalar(topH_MIN,topS_MIN,topV_MIN),Scalar(topH_MAX,topS_MAX,topV_MAX),topThreshold);
			inRange(frame0_hsv_warped,Scalar(botH_MIN,botS_MIN,botV_MIN),Scalar(botH_MAX,botS_MAX,botV_MAX),botThreshold);


           	toggle = 0;
	}
 

	//Compute the absolute value of the difference
	cv::cuda::absdiff(gpu_grayImage0, gpu_grayImage1, gpu_differenceImage);


	// Threshold the difference image
    cv::cuda::threshold(gpu_differenceImage, gpu_thresholdImage, 70, 255, cv::THRESH_BINARY);
	gpu_thresholdImage.download(result);


	// Find the location of any moving object and show the final frame
	if (toggle == 0) {
            searchForMovement(result,frame0_warped);
			//searchForPaddles(top_roi, topThreshold, frame0_warped);
			searchForPaddles(bot_roi, botThreshold, frame0_warped);


			cout << "return point frame0:                  " << currentLoc << endl;
			circle(frame0_warped, Point(currentLoc, roi_ymin+roi_height), 5,cv::Scalar(0,255,0),2);
			circle(frame0_warped, Point(returnLoc, roi_ymin+roi_height), 5,cv::Scalar(0,255,0),2);
			line(frame0, inputQuad[0], inputQuad[1], Scalar(0,0,255),2);
			line(frame0, inputQuad[1], inputQuad[2],Scalar(0,0,255),2);
			line(frame0, inputQuad[2], inputQuad[3] ,Scalar(0,0,255),2);
			line(frame0, inputQuad[3], inputQuad[0],Scalar(0,0,255),2);
			//findCorners(gameThreshold, frame0);
	        //imshow("Frame", frame0);
			//imshow("FrameWarp", frame0_warped);
			//imshow("HSV", frame0_hsv_warped);
			imshow("threshold", botThreshold);
			//imshow("threshold", gameThreshold);
			//imshow("threshold",result);

	}
	else {
			searchForMovement(result,frame1_warped);
			//searchForPaddles(top_roi, topThreshold, frame1_warped);
			searchForPaddles(bot_roi, botThreshold, frame1_warped);
			cout << "return point frame1:                  " << currentLoc << endl;
			circle(frame1_warped, Point(currentLoc, roi_ymin+roi_height), 5,cv::Scalar(0,255,0),2);
			circle(frame1_warped, Point(returnLoc, roi_ymin+roi_height), 5,cv::Scalar(0,255,0),2);
			line(frame1, inputQuad[0], inputQuad[1], Scalar(0,0,255),2);
			line(frame1, inputQuad[1], inputQuad[2],Scalar(0,0,255),2);
			line(frame1, inputQuad[2], inputQuad[3] ,Scalar(0,0,255),2);
			line(frame1, inputQuad[3], inputQuad[0],Scalar(0,0,255),2);
			//findCorners(gameThreshold, frame1);
	        //imshow("Frame", frame1);
			//imshow("FrameWarp", frame1_warped);
			//imshow("HSV", frame1_hsv_warped);
			imshow("threshold", botThreshold);
			//imshow("threshold", gameThreshold);
			//imshow("threshold",result);

	}


//-----------------------------------------------------------------------------------------------
// Drive Paddle to Correct Location Stuff
//-----------------------------------------------------------------------------------------------

char right = 'R';
char left = 'L';
char serve = 'S';
int dur = 8;
	
	difference = currentLoc - returnLoc;
	if(difference < 0){
		writeArduino(right, dur);
		cout << "Arduino go right" << endl;
	}
	else if(difference > 0){
		writeArduino(left, dur);
		cout << "Arduino go left" << endl;
	}
	if(disappeared){
		writeArduino(right,dur);
		cout << "Arduino go left" << endl;
	}
	cout << "Culoc:   " << currentLoc << endl;
	cout << "Reloc:   " << returnLoc << endl;
	cout << "error:   " << difference << endl << endl;


	frame_count++;
	if((frame_count % 10) == 0){
		cout << "Check Perspective Change" << endl;	
		inputQuad[0] = Point2f(lambda1X,lambda1Y);
		inputQuad[1] = Point2f(lambda2X,lambda2Y);
		inputQuad[2] = Point2f(lambda3X,lambda3Y);
		inputQuad[3] = Point2f(lambda4X,lambda4Y);

		outputQuad[3] = Point2f(437,0);
		outputQuad[0] = Point2f(842,0);
		outputQuad[1] = Point2f(842,719);
		outputQuad[2] = Point2f(437,719);
		lambda = cv::getPerspectiveTransform(inputQuad,outputQuad);

		writeArduino(serve,20);
	}

	cv::waitKey(1); //needed to show frame
    }
close(fd);
}
