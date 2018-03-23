#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string> 
#include <sstream>
#include <stdio.h>
#include "lsf.h"

using namespace cv;
using namespace std;

lsf::lsf(cv::Rect roi, int array_size){
	m_array_size = array_size;
	m_roi = roi;
	m_x_slope = 0;
	m_y_slope = 0;
	m_sum_x = 0;
	m_Xbar = 0;	
	m_sum_y = 0;
	m_Ybar = 0;
	m_xmin = m_roi.x;
	m_xmax = m_roi.x+m_roi.width;
	m_ymin = m_roi.y;
	m_ymax = m_roi.y+m_roi.height;
}

unsigned int lsf::getSize(){
	//cout << "Get Size Function called." <<  endl;
	return m_dq.size();
}


bool lsf::addPoint(cv::Point2i point){
	m_sum_x = m_sum_x + point.x;
	m_sum_y = m_sum_y + point.y;
	m_dq.push_front(point);
	m_Xbar = m_sum_x/m_dq.size();
	m_Ybar = m_sum_y/m_dq.size();
		
	//Determine if ball is moving left or right - could be useful
	if(m_dq.size() >= m_array_size){
		m_xdir_prev = m_xdir;
		if((m_dq.at(0).x - m_dq.at(m_array_size-1).x) < 0){
			m_xdir = 0;
			cout << "Ball moving left"  << endl;
		}else{
			m_xdir = 1;
			cout << "Ball moving right"  << endl;
		}
	}

	//Determine if ball is moving up or down - could be useful
	if(m_dq.size() >= m_array_size){
		m_xdir_prev = m_xdir;
		if((m_dq.at(0).y - m_dq.at(m_array_size-1).y) < 0){
			m_ydir = 0;
			cout << "Ball moving up"  << endl;
		}else{
			m_ydir = 1;
			cout << "Ball moving down"  << endl;
		}
	}
	cout << "All Points:   "  << endl;
	for(int i = 0; i < m_dq.size(); i++){
		cout << m_dq.at(i) << endl;
	}
	cout << "The size of the deck is:  " << m_dq.size() << endl;
	cout << "X sum:  "  << m_sum_x << endl;
	cout << "Xbar:  "  << m_Xbar << endl;
	cout << "Y sum:  "  << m_sum_y << endl;
	cout << "Ybar:  "  << m_Ybar << endl;
return m_ydir;
}



bool lsf::testMonotonic(){
	bool monotonic_x;
	bool monotonic_y;
	bool monotonic;

/*	if(m_xdir == m_xdir_prev){
		monotonic = 1;
	} else{
		monotonic = 0;	
	}
*/

	if((m_dq.at(0).x - m_dq.at(1).x) >= 0){
		cout << "Decreasing..." << endl;
		for(int i = 0; i < m_array_size; i++){
			if((m_dq[i].x - m_dq[i+1].x) >= -2){
				monotonic_x = 1;
			}else{
				monotonic_x = 0;
				break;
			}
		}
	}else if((m_dq.at(0).x - m_dq.at(1).x) < 0){
		cout << "Increasing..." << endl;
		for(int i = 0; i < m_array_size; i++){
			if((m_dq[i].x - m_dq[i+1].x) <= 2){
				monotonic_x = 1;
			}else{
				monotonic_x = 0;
				break;
			}
		}
	}else{
		monotonic_x = 0;	
	}cout << "Monotonic in X?...  " << monotonic_x << endl;


	if((m_dq[0].y - m_dq[1].y) >= 0){
		cout << "Decreasing..." << endl;
		for(int i = 0; i < m_array_size; i++){
			if((m_dq[i].y - m_dq[i+1].y) >= -2){
				monotonic_y = 1;
			}else{
				monotonic_y = 0;
				break;
			}
		}
	}else if((m_dq[0].y - m_dq[1].y) < 0){
		cout << "Increasing..." << endl;
		for(int i = 0; i < m_array_size; i++){
			if((m_dq[i].y - m_dq[i+1].y) <= 2){
				monotonic_y = 1;
			}else{
				monotonic_y = 0;
				break;
			}
		}
	}else{
		monotonic_y = 0;	
	}cout << "Monotonic in Y?...  " << monotonic_y  << endl;

if(monotonic_x && monotonic_y){
		monotonic = 1;
	}else{
		monotonic = 0;
}

cout << "Monotonic in x?...  " << monotonic << endl << endl;
return monotonic;
}

void lsf::computeLSF(){
	float denom_LSF = 0;
	float num_LSF_x = 0;
	float num_LSF_y = 0;
	float size = m_dq.size();
	
	//Denominator of LSF for a given array size:
	for (int i = 0; i < m_dq.size(); i++){
		denom_LSF += (((float)i-(size - 1)/2) * ((float)i-(size - 1)/2));
	}
	cout << "COMPUTELSF: Got Denom" << endl;
	//Numerator
	for(int i = 0; i < size; i++){
		num_LSF_x += ((float)i-(size-1)/2)*(m_dq.at(i).x - m_Xbar);
		num_LSF_y += ((float)i-(size-1)/2)*(m_dq.at(i).y - m_Ybar);
	}	
	cout << "COMPUTELSF: Got num" << endl;
	m_x_slope = -(num_LSF_x/denom_LSF);
	m_y_slope = -(num_LSF_y/denom_LSF);
	cout << "LSF denominator is:  " << denom_LSF << endl;
	cout << "X slope is:  " << m_x_slope << endl;
	cout << "Y slope is:  " << m_y_slope << endl << endl;
}


int lsf::computeRebounds(cv::Point2i point){
	cout << "computeRebounds Called  " << endl;
	cout << "roi X:  " << m_xmin << " " << m_xmax << endl;
	cout << "roi Y:  " << m_ymin << " " << m_ymax << endl;
	float t_x;
	float t_y;
	int x_collision;
	int y_collision;
	int n_rebounds = 1;
	m_rebounds.clear();
	m_rebounds.push_front(point);
while(1){

if(m_rebounds.empty())
cout << "Deque is Empty" << endl;

	cout << "Using Point:  " << m_rebounds.front() << endl;
	if(m_x_slope == 0){
		break;	
	}
	else if(m_x_slope > 0){
		cout << "Died line 191  "  << endl;
		t_x = (m_xmax - m_rebounds.front().x - m_xmin)/m_x_slope;
	}else{
		cout << "Died line 194  "  << endl;
		t_x = ((-m_rebounds.front().x)/m_x_slope);
	}
	if(m_y_slope >= 0){
		cout << "Died line 198  " << endl;
		t_y = ((m_ymax - (m_rebounds.front().y + m_ymin))/m_y_slope);	
	}else {
		cout << "Died line 201  " << endl;
		t_y = ((-m_rebounds.front().y)/m_y_slope);	
	}
	cout << "TX:  " << t_x << endl;
	cout << "TY:  " << t_y << endl;
	if( (t_x) > (t_y) ){
		if (m_ydir){
			cout << "GOING TO HIT BOTTOM - out of loop" << endl;
			x_collision = (m_x_slope*t_y) + (m_rebounds.front().x);
			m_rebounds.push_front(Point2i(x_collision, m_roi.height));
			n_rebounds++;
			cout << "number of rebounds  " << n_rebounds << endl;
			//m_ydir = !m_ydir;
			break;
		}else{
			cout << "GOING TO HIT TOP - out of loop" << endl;
			x_collision = (m_x_slope*t_y) + (m_rebounds.front().x);
			m_rebounds.push_front(Point2i(x_collision, 0));
			n_rebounds++;
			cout << "number of rebounds  " << n_rebounds << endl;
			//m_ydir = !m_ydir;
			break;
		}	
	}else{
		if (m_xdir){
			cout << "GOING TO HIT RIGHT" << endl;
			y_collision = (m_y_slope * t_x) + (m_rebounds.front().y);
			m_rebounds.push_front(Point2i(m_roi.width, y_collision));
			n_rebounds++;
			cout << "number of rebounds  " << n_rebounds << endl;
			m_x_slope = -m_x_slope;
			m_xdir = !m_xdir;
		}else{
			cout << "GOING TO HIT LEFT" << endl;
			y_collision = (m_y_slope * t_x) + (m_rebounds.front().y);
			m_rebounds.push_front(Point2i(0, y_collision));
			n_rebounds++;
			cout << "number of rebounds  " << n_rebounds << endl;
			m_x_slope = -m_x_slope;
			m_xdir = !m_xdir;		
		}	
	}
	cout << "X slope:  " << m_x_slope << endl;
	cout << "Y slope:  " << m_y_slope << endl;

	/*for(int i = 0; i < (n_rebounds); i++){
		cout << m_rebounds.at(i) << endl;
	}
*/
	if(n_rebounds >= 6){
		break;	
	}
	

	}//End While
return n_rebounds;
}

cv::Point2i lsf::getRebound(int n){
	cv::Point2i returnPoint;
	returnPoint.x = m_rebounds.at(n).x + m_xmin;
	returnPoint.y = m_rebounds.at(n).y + m_ymin;
return returnPoint;
}

void lsf::trimPoints(int n){
	for(int i = 0; i<(m_dq.size()-n); i++){
		m_sum_x = m_sum_x - m_dq.at(m_dq.size()-1).x;
		m_sum_y = m_sum_y - m_dq.at(m_dq.size()-1).y;
		m_dq.pop_back();
	}
	cout << "All Points:   "  << endl;
	for(int i = 0; i < m_dq.size(); i++){
		cout << m_dq.at(i) << endl;
	}
}

int lsf::getSecondToLastPoint(){
	cout << "Second to Last Rebound:  " << m_rebounds.at(m_rebounds.size()-1).y << endl;
	return (m_rebounds.at(m_rebounds.size()-1).y);
}
