#include "fingerCount.h"


namespace {
	void help(char** av) {
		cout << "\nThis program justs gets you started reading images from video\n"
			"Usage:\n./" << av[0] << " <video device number>\n"
			<< "q,Q,esc -- quit\n"
			<< "space   -- save frame\n\n"
			<< "\tThis is a starter sample, to get you up and going in a copy pasta fashion\n"
			<< "\tThe program captures frames from a camera connected to your computer.\n"
			<< "\tTo find the video device number, try ls /dev/video* \n"
			<< "\tYou may also pass a video file, like my_vide.avi instead of a device number"
			<< endl;
	}



	int process(VideoCapture& capture) {
		int n = 0;
		char filename[200];
		string window_name = "video | q or esc to quit";
		cout << "press space to save a picture. q or esc to quit" << endl;
		namedWindow(window_name, CV_WINDOW_KEEPRATIO); //resizable window;
		Mat frame;
		Mat output;
		int counter = 0;
		for (;;) {
			counter++;
			capture >> frame;
			if (frame.empty())
				continue;

			fingerCount(frame);
			imshow(window_name, frame);

			char key = (char)waitKey(5); //delay N millis, usually long enqough to display and capture input
			switch (key) {
			case 'q':
			case 'Q':
			case 27: //escape key
				return 0;
			case ' ': //Save an image
				sprintf(filename,"filename%.3d.jpg",n++);
				imwrite(filename,frame);
				cout << "Saved " << filename << endl;
				break;
			default:
				break;
			}
		}
		return 0;
	}

}

int main(int ac, char** av) {

	if (ac != 2) {
		help(av);
		return 1;
	}
	std::string arg = av[1];
	VideoCapture capture(0); //try to open string, this will attempt to open it as a video file
	if (!capture.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
		capture.open(atoi(arg.c_str()));
	if (!capture.isOpened()) {
		cerr << "Failed to open a video device or video file!\n" << endl;
		help(av);
		return 1;
	}
	return process(capture);
}