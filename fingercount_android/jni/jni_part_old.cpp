#include <jni.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <sstream>

using namespace std;
using namespace cv;

extern "C" {
JNIEXPORT void JNICALL Java_org_opencv_samples_tutorial4_Sample4View_FindFeatures(JNIEnv* env, jobject thiz, jlong addrGray, jlong addrRgba)
{
	cv::Mat *matImage = (Mat*)addrRgba;
	IplImage* image = cvCreateImage(cvSize((*matImage).cols,(*matImage).rows ), 8, 3);
	image->imageData = (char*)(*matImage).data;

	IplImage * grayImg = cvCreateImage( cvSize(image->width, image->height), IPL_DEPTH_8U, 1 );
	IplImage * hsvImg = cvCreateImage( cvSize(image->width, image->height), IPL_DEPTH_8U, 3 );

	cvCvtColor( image, hsvImg, CV_BGR2HSV );

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

	//cvErode( grayImg, grayImg, NULL, 1);
	//cvDilate( grayImg, grayImg, NULL, 2);

	cvDilate( grayImg, grayImg, NULL, 2);
	//cvErode( grayImg, grayImg, NULL, 1);

	//cv::Mat *matImage2 = (Mat*)addrGray;
	//(char*)(*grayImage).data = grayImg->imageData;
	//(char*)( (*matImage2).data) = grayImg->imageData;


	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* contours = 0;
	CvSeq* contoursTemp = 0;
	int header_size = sizeof(CvContour);
	//cvThreshold( grayImg, grayImg, 30, 255, CV_THRESH_BINARY );

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
		area = cvContourArea( hand_contour, CV_WHOLE_SEQ);
		cvDrawContours(image, hand_contour, CV_RGB(0, 255, 0), CV_RGB(0, 0, 0), 2, 2, 8);

		CvRect rect;
		//int count = contours->total;
		//rect = cvContourBoundingRect( hand_contour, 1);
		rect = cvBoundingRect( hand_contour, 1);

		cvRectangle(image,
			cvPoint(rect.x,rect.y),
			cvPoint(rect.x + rect.width,rect.y + rect.height),
			CV_RGB(255,255,255) ,
			1);

		int meanLength = (rect.height+rect.width)/10;

		seqhull = cvConvexHull2(hand_contour, NULL, CV_COUNTER_CLOCKWISE, 0);

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
				if( startPt.y < depthPt.y &&
						sqrtf( (startPt.x - depthPt.x)*(startPt.x - depthPt.x) + (startPt.y - depthPt.y)*(startPt.y - depthPt.y)) > meanLength)
				{
					numFingers++;
					//cvLine(image, *(defectArray[i].start),
					//	*(defectArray[i].depth_point),
					//	CV_RGB(255,255,255),
					//	3,
					//	8,
					//	0 );
					//cvCircle( image,*(defectArray[i].depth_point),5,CV_RGB(255,255,0),3,8,0);
					cvCircle( image,*(defectArray[i].start),5,CV_RGB(0,255,255),3,8,0);
					cvLine(image,*(defectArray[i].depth_point),*(defectArray[i].start),CV_RGB(255,0,255),3,8,0 );

					//cvCircle( image, *(defectArray[i].end), 5, CV_RGB(255,0,0), -1, 8,0);
					//cvCircle( image, *(defectArray[i].start), 5, CV_RGB(0,0,255), -1, 8,0);
					//cvCircle( image, *(defectArray[i].depth_point), 5, CV_RGB(0,0,255), -1, 8,0);
				}

			}
		}
		if(numFingers > 0){
			char buffer [5];
			std::ostringstream sin;
			sin << numFingers;
			std::string val = sin.str();
			CvFont font;
			double hScale=3.0;
			double vScale=3.0;
			int    lineWidth=2;
			cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
			cvPutText(image, val.c_str(),cvPoint(rect.x, rect.y), &font, cvScalar(255,100,100));
			//cout<<"Fingers: "<<numFingers<<endl;

		}
	}

	hand_contour = 0;
	cvReleaseImage(&grayImg);
	cvReleaseImage(&hsvImg);

}

}


