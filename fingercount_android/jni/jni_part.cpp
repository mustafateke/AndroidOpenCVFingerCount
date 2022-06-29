#include <jni.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <sstream>
#include <time.h>

using namespace std;
using namespace cv;

extern "C" {
JNIEXPORT void JNICALL Java_org_opencv_samples_tutorial4_Sample4View_FindFeatures(JNIEnv* env, jobject thiz, jlong addrGray, jlong addrRgba)
{

	clock_t starttime = clock();

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

	clock_t endtime = clock();
	int timedif = (endtime - starttime);
	std::ostringstream sin;
	sin << timedif <<" usec";
	std::string val = sin.str();

	CvFont font;
	double hScale=1.0;
	double vScale=1.0;
	int    lineWidth=3;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
	cvPutText(image, val.c_str(),cvPoint(50, 50), &font, cvScalar(100,100,100));


	hand_contour = 0;
	cvReleaseImage(&grayImg);
	cvReleaseImage(&hsvImg);

}

}

extern "C" {
JNIEXPORT void JNICALL Java_org_opencv_samples_tutorial4_Sample4View_OpticalFlow(JNIEnv* env, jobject thiz, jlong addrImgA, jlong addrImgB)
{

	clock_t starttime = clock();

	const int MAX_CORNERS = 1000;

	int         win_size = 10;

	cv::Mat *matImageA = (Mat*)addrImgA;
	cv::Mat *matImageB = (Mat*)addrImgB;

	cv::Mat imgA;
	cv::Mat imgB;
	cvtColor( *matImageA, imgA, CV_BGR2GRAY );
	cvtColor( *matImageB, imgB, CV_BGR2GRAY );


	TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
	Size subPixWinSize(10,10), winSize(10,10);

	vector<Point2f> points[2];

	//goodFeaturesToTrack(imgB, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
	vector<KeyPoint> keypoints;
	Ptr<FeatureDetector> fd = FeatureDetector::create("FAST");
	/*cv::FAST(imgB, keypoints, 25);*/
	fd->detect(imgB, keypoints);
	for(int i = 0; i < keypoints.size() && i< MAX_CORNERS; i++){
		points[1].push_back(keypoints[i].pt);
	}
	//cornerSubPix(imgB, points[1], subPixWinSize, Size(-1,-1), termcrit);

	vector<uchar> status;
	vector<float> err;

	calcOpticalFlowPyrLK(imgA, imgB, points[1], points[0], status, err, winSize,
		3, termcrit, 0, 0, 0.001);


	for (int i = 0; i < points[0].size(); i++)
	{
		cv::line(*matImageB, points[0][i], points[1][i], Scalar(0,255,0));
	}

	clock_t endtime = clock();
	int timedif = endtime - starttime;
	std::ostringstream sin;
	sin << timedif <<" usec";
	std::string val = sin.str();

	cv::putText(*matImageB, val.c_str(), cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(100,100,100), 3);

}
}

extern "C" {
JNIEXPORT void JNICALL Java_org_opencv_samples_tutorial4_Sample4View_OpticalFlow2(JNIEnv* env, jobject thiz, jlong addrImgA, jlong addrImgB)
{

	clock_t starttime = clock();
	const int MAX_CORNERS = 1000;

	int         win_size = 10;

	cv::Mat *matImageA = (Mat*)addrImgA;
	cv::Mat *matImageB = (Mat*)addrImgB;


	IplImage* image0 = cvCreateImage(cvSize((*matImageA).cols,(*matImageA).rows ), 8, 3);
	image0->imageData = (char*)(*matImageA).data;
	IplImage* image = cvCreateImage(cvSize((*matImageB).cols,(*matImageB).rows ), 8, 3);
	image->imageData = (char*)(*matImageB).data;


	IplImage * imgA = cvCreateImage( cvSize(image->width, image->height), IPL_DEPTH_8U, 1 );
	IplImage * imgB = cvCreateImage( cvSize(image->width, image->height), IPL_DEPTH_8U, 1 );
	cvCvtColor( image0, imgA, CV_BGR2GRAY );
	cvCvtColor( image, imgB, CV_BGR2GRAY );

	//cvCvtColor( image, image, CV_BGR2HSV );
	// The first thing we need to do is get the features
	// we want to track.
	//

	IplImage* eig_image = cvCreateImage( cvSize(image->width, image->height), IPL_DEPTH_32F, 1 );
	IplImage* tmp_image = cvCreateImage( cvSize(image->width, image->height), IPL_DEPTH_32F, 1 );
	int              corner_count = MAX_CORNERS;
	CvPoint2D32f* cornersA        = new CvPoint2D32f[ MAX_CORNERS ];

	vector<KeyPoint> keypoints;
	Ptr<FeatureDetector> fd = FeatureDetector::create("FAST");
	/*cv::FAST(imgB, keypoints, 25);*/
	fd->detect(imgB, keypoints);
	int i;
	for(i=0;i<keypoints.size() && i< MAX_CORNERS;i++){
		cornersA[i] = cvPoint2D32f(keypoints[i].pt.x,keypoints[i].pt.y);
	}

	corner_count = i;

	/*
	cvGoodFeaturesToTrack(
		imgA,
		eig_image,
		tmp_image,
		cornersA,
		&corner_count,
		0.01,
		5.0,
		0,
		3,
		0,
		0.04
	);
	cvFindCornerSubPix(
		imgA,
		cornersA,
		corner_count,
		cvSize(win_size,win_size),
		cvSize(-1,-1),
		cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03)
	);
	*/
	// Call the Lucas Kanade algorithm
	//
	char features_found[ MAX_CORNERS ];
	float feature_errors[ MAX_CORNERS ];
	CvSize pyr_sz = cvSize( imgA->width+8, imgB->height/3 );
	IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  CvPoint2D32f* cornersB        = new CvPoint2D32f[ MAX_CORNERS ];
  cvCalcOpticalFlowPyrLK(
     imgA,
     imgB,
     pyrA,
     pyrB,
     cornersA,
     cornersB,
     corner_count,
     cvSize( win_size,win_size ),
     5,
     features_found,
     feature_errors,
     cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),
     0
  );
  // Now make some image of what we are looking at:
  //
  for( int i=0; i<corner_count; i++ ) {
     if( features_found[i]==0|| feature_errors[i]>550 ) {
 //       printf("Error is %f/n",feature_errors[i]);
        continue;
     }
 //    printf("Got it/n");
     CvPoint p0 = cvPoint(
        cvRound( cornersA[i].x ),
        cvRound( cornersA[i].y )
     );
     CvPoint p1 = cvPoint(
        cvRound( cornersB[i].x ),
        cvRound( cornersB[i].y )
     );
     cvLine( image, p0, p1, CV_RGB(255,0,0),2 );
  }

	clock_t endtime = clock();
	int timedif = endtime - starttime;
	std::ostringstream sin;
	sin << timedif <<" usec";
	std::string val = sin.str();

	CvFont font;
	double hScale=1.0;
	double vScale=1.0;
	int    lineWidth=3;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
	cvPutText(image, val.c_str(),cvPoint(50, 50), &font, cvScalar(255,100,100));

}
}
