package org.opencv.samples.tutorial4;

import java.util.ArrayList;
import java.util.List;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.CvType;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.KeyPoint;
import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;

import android.content.Context;
import android.graphics.Bitmap;
import android.os.Environment;
import android.view.SurfaceHolder;

class Sample4View extends SampleViewBase {
	private Mat mYuv;
	private Mat mRgba;
	private Mat mRgb;
	private Mat mRgb_old;
	private Mat mGraySubmat;
	private Mat mIntermediateMat;

	private int counter = 0;

	public Sample4View(Context context) {
		super(context);
	}

	@Override
	public void surfaceChanged(SurfaceHolder _holder, int format, int width, int height) {
		super.surfaceChanged(_holder, format, width, height);

		synchronized (this) {
			// initialize Mats before usage
			mYuv = new Mat(getFrameHeight() + getFrameHeight() / 2, getFrameWidth(), CvType.CV_8UC1);
			mGraySubmat = mYuv.submat(0, getFrameHeight(), 0, getFrameWidth());

			mRgba = new Mat();
			mRgb = new Mat();
			mIntermediateMat = new Mat();
		}
	}

	@Override
	protected Bitmap processFrame(byte[] data) {
		mYuv.put(0, 0, data);
		counter ++;

		switch (Sample4Mixed.viewMode) {
		case Sample4Mixed.VIEW_MODE_GRAY:


			long start = System.nanoTime(); 
			
			Imgproc.cvtColor(mYuv, mRgba, Imgproc.COLOR_YUV420sp2RGB, 4);

			int MAX_CORNERS = 1000;

			Imgproc.cvtColor(mRgba, mRgb, Imgproc.COLOR_RGBA2BGR, 4);
//			Mat imgA = new Mat();
//			Mat imgB = new Mat();
//			
//			Imgproc.cvtColor(mRgb, imgA, Imgproc.COLOR_BGR2GRAY);
//			Imgproc.cvtColor(mRgb, imgB, Imgproc.COLOR_BGR2GRAY);
//			
//			List<KeyPoint> keypoints = new ArrayList<KeyPoint>();
//			List<Point> prevPts = new ArrayList<Point>();
//			List<Point> nextPts = new ArrayList<Point>();	
//			List<Byte> status = new ArrayList<Byte>();
//			List<Float> err = new ArrayList<Float>();
//			
//			FeatureDetector fastdetect =  FeatureDetector.create(FeatureDetector.FAST);
//			fastdetect.detect(imgB, keypoints);
//			
//			for(int i = 0; i < keypoints.size() && i < MAX_CORNERS; i++)
//			{
//				nextPts.add(keypoints.get(i).pt);
//			}
//			
//			for(int i = 0; i < keypoints.size() && i < MAX_CORNERS; i++)
//			{
//				prevPts.add(keypoints.get(i).pt);
//			}			
//			
////			//calcOpticalFlowPyrLK(imgA, imgB, prevPts, nextPts, status, err);
//			Video.calcOpticalFlowPyrLK(imgA, imgB,  prevPts,  nextPts,  status,  err);
////			
////			for (int i = 0; i < nextPts.size() && i < prevPts.size(); i++)
////			{
////				Core.line(mRgb, prevPts.get(i), nextPts.get(i), new Scalar(255, 100,100));
////			}
//			
//			for (int i = 0; i < nextPts.size() ; i++)
//			{
//				Core.circle(mRgb, nextPts.get(i), 1, new Scalar(255, 100,100));
//			}			

			OpticalFlow(mRgb.getNativeObjAddr(), mRgb.getNativeObjAddr());  
			
			//long estimatedTime = System.nanoTime() - start;
			
			//Core.putText(mRgb, estimatedTime/1000 + " usec", new Point(50,50), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(100, 255,100));

			Imgproc.cvtColor(mRgb, mRgba, Imgproc.COLOR_BGR2RGBA, 4);

			break;
		case Sample4Mixed.VIEW_MODE_RGBA:
			Imgproc.cvtColor(mYuv, mRgba, Imgproc.COLOR_YUV420sp2RGB, 4);
			break;
		case Sample4Mixed.VIEW_MODE_CANNY:
			Imgproc.Canny(mGraySubmat, mIntermediateMat, 80, 100);
			Imgproc.cvtColor(mIntermediateMat, mRgba, Imgproc.COLOR_GRAY2BGRA, 4);
			break;
case Sample4Mixed.VIEW_MODE_FINGERCOUNT:
	Imgproc.cvtColor(mYuv, mRgba, Imgproc.COLOR_YUV420sp2RGB, 4);
	Imgproc.cvtColor(mRgba, mRgb, Imgproc.COLOR_RGBA2BGR, 4);
	FindFeatures(mGraySubmat.getNativeObjAddr(), mRgb.getNativeObjAddr());
	Imgproc.cvtColor(mRgb, mRgba, Imgproc.COLOR_BGR2RGBA, 4);
	break;
		}

		if( counter == 100)
		{
			Highgui.imwrite("/mnt/sdcard/result.jpg", mRgb);
		}
		Bitmap bmp = Bitmap.createBitmap(getFrameWidth(), getFrameHeight(), Bitmap.Config.ARGB_8888);

		if (Utils.matToBitmap(mRgba, bmp))
			return bmp;

		bmp.recycle();
		return null;
	}

	@Override
	public void run() {
		super.run();

		synchronized (this) {
			// Explicitly deallocate Mats
			if (mYuv != null)
				mYuv.release();
			if (mRgba != null)
				mRgba.release();
			if (mRgb_old != null)
				mRgb_old.release();            
			if (mRgb != null)
				mRgb.release();            
			if (mGraySubmat != null)
				mGraySubmat.release();
			if (mIntermediateMat != null)
				mIntermediateMat.release();

			mYuv = null;
			mRgba = null;
			mRgb_old = null;
			mRgb = null;
			mGraySubmat = null;
			mIntermediateMat = null;
		}
	}
	

	public native void FindFeatures(long matAddrGr, long matAddrRgba);
	public native void OpticalFlow(long addrImgA, long addrImgB);
	public native void OpticalFlow2(long addrImgA, long addrImgB);

	static {
		System.loadLibrary("mixed_sample");
	}
}
