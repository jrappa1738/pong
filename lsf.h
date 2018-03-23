#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string> 
#include <sstream>
#include <stdio.h>

class lsf{
	private: 
		std::deque<cv::Point2i> m_dq;
		std::deque<cv::Point2i> m_rebounds;
		cv::Rect m_roi;
		double m_x_slope;
		double m_y_slope;
		int m_array_size;
		double m_sum_x;
		double m_Xbar;
		double m_sum_y;
		double m_Ybar;
		bool m_xdir, m_xdir_prev;
		bool m_ydir, m_ydir_prev;
		int m_xmin, m_xmax, m_ymin, m_ymax;

	public:
		lsf(cv::Rect roi, int array_size);
		unsigned int getSize();
		bool addPoint(cv::Point2i point);
		bool testMonotonic();
		void computeLSF();
		int computeRebounds(cv::Point2i point);
		cv::Point2i getRebound(int n);
		void trimPoints(int n);
		int getSecondToLastPoint();
};
