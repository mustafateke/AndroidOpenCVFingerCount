#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int fingerCount(cv::Mat matImage)
{
	IplImage* image = cvCreateImage(cvSize(matImage.cols,matImage.rows ), 8, 3);
	image->imageData = (char*)matImage.data; 

	IplImage * grayImg = cvCreateImage( cvSize(image->width, image->height), IPL_DEPTH_8U, 1 );
	IplImage * hsvImg = cvCreateImage( cvSize(image->width, image->height), IPL_DEPTH_8U, 3 );

	/// Obtain HSV Image
	cvCvtColor( image, hsvImg, CV_BGR2HSV );

	/// Find Hand from HSV Image
	int width     = hsvImg->width;
	int height    = hsvImg->height;
	for( int i = 0 ; i < height ; i++ ) {
		for( int j = 0 ; j < width ; j++ ) {
			uchar h = ((uchar *)(hsvImg->imageData + i*hsvImg->widthStep))[j*hsvImg->nChannels + 0];
			uchar s = ((uchar *)(hsvImg->imageData + i*hsvImg->widthStep))[j*hsvImg->nChannels + 1];
			uchar v = ((uchar *)(hsvImg->imageData + i*hsvImg->widthStep))[j*hsvImg->nChannels + 2];

			(grayImg->imageData + i*grayImg->widthStep)[j] = v;

			if( h < 20 && s > 45 && v > 80){
				(grayImg->imageData + i*grayImg->widthStep)[j*grayImg->nChannels] = uchar(255);
			}
			else
			{
				(grayImg->imageData + i*grayImg->widthStep)[j*grayImg->nChannels] = uchar(0);
			}
		}
	}

	cvDilate( grayImg, grayImg, NULL, 2);

	cvNamedWindow( "in", 1 );
	cvShowImage("in",grayImg);

	cvWaitKey(5);
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* contours = 0;
	CvSeq* contoursTemp = 0;
	int header_size = sizeof(CvContour);

	/// Find Hand Contours
	int c = cvFindContours(grayImg,
		storage,
		&contours,
		sizeof(CvContour),
		CV_RETR_TREE,CV_CHAIN_APPROX_NONE,
		cvPoint(0,0));

	CvPoint* PointArray;
	CvSeq *seqhull, *defects;
	CvMemStorage *stor03 = cvCreateMemStorage(0);
	CvConvexityDefect *defectArray;

	double maxArea = 0;
	CvSeq* hand_contour = NULL;

	double area=0,biggestArea=0;

	///Find Largest Contour as the hand
	while(contours)
	{
		area = fabs( cvContourArea( contours, CV_WHOLE_SEQ ) );

		if ( area > biggestArea) 
		{
			biggestArea = area; 
			hand_contour = contours;
		};

		contours  =  contours->h_next;
	}

	if (hand_contour)
	{
		CvRect rect;
		rect = cvBoundingRect( hand_contour, 1);

		cvRectangle(image,
			cvPoint(rect.x-10,rect.y-10), 
			cvPoint(rect.x + rect.width+10,rect.y + rect.height+10),
			CV_RGB(255,255,255) ,
			1);

		int meanLength = (rect.height+rect.width)/8;

		seqhull = cvConvexHull2(hand_contour, NULL, CV_COUNTER_CLOCKWISE, 0);

		/// Obtain Defects
		defects = cvConvexityDefects( hand_contour, seqhull, stor03);
		int nomdef = defects->total;

		defectArray = (CvConvexityDefect*)malloc(sizeof(CvConvexityDefect)*nomdef);
		cvCvtSeqToArray(defects,defectArray, CV_WHOLE_SEQ);

		int numFingers = 0;
		for(;defects;defects = defects->h_next)
		{
			for(int i=0; i<nomdef; i++)
			{
				CvPoint depthPt = *(defectArray[i].depth_point);
				CvPoint startPt = *(defectArray[i].start);
				CvPoint endPt = *(defectArray[i].end);

				/// Filter Finger Defects
				if( startPt.y < depthPt.y &&
					sqrtf( (startPt.x - depthPt.x)*(startPt.x - depthPt.x) + (startPt.y - depthPt.y)*(startPt.y - depthPt.y)) > meanLength)
				{
					numFingers++;
					/// Draw Finger Points
					cvCircle( image,*(defectArray[i].start),5,CV_RGB(0,255,255),3,8,0);

					/// Draw Finger Lines
					cvLine(image,*(defectArray[i].depth_point),*(defectArray[i].start),CV_RGB(255,0,255),3,8,0 );
				}

			}
		}

		/// Put number of fingers on screen
		if(numFingers > 0){
			std::ostringstream sin;
			sin << numFingers;
			std::string val = sin.str();
			CvFont font;
			double hScale=1.0;
			double vScale=1.0;
			int    lineWidth=1;
			cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
			cvPutText(image, val.c_str(),cvPoint(rect.x, rect.y), &font, cvScalar(255,100,100));
			return numFingers;
		}
	}

	hand_contour = 0;

	return 0;
}