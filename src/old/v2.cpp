#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <ctime>

using namespace cv;
using namespace std;

float dist(Point2f p1, Point2f p2) {
	float dx = (p1.x - p2.x);
	float dy = (p1.y - p2.y);
	return sqrt(dx * dx + dy * dy);
}

//void drawFps(Mat img) {
//	static clock_t lastFrameTime;
//	clock_t thisFrameTime = clock();
//	float dt = (float)(thisFrameTime - lastFrameTime) / CLOCKS_PER_SEC;
//	lastFrameTime = thisFrameTime;
//	putText(img, to_string(1 / dt), Point(20, 40),
//		CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 255));
//}

int main(int argc, char** argv)
{
	int width = 640;//960;
	int height = 480;//540;
	int roiWidth = 300;
	Rect roi[3];
	roi[0] = Rect((width - roiWidth) / 2, 0, roiWidth, height);
	roi[1] = Rect(0, 0, roiWidth, height);
	roi[2] = Rect(width - roiWidth, 0, roiWidth, height);

	VideoCapture cap;
	//if (!cap.open("C:/Users/tae/Stuffs/Study/4_1_2110433 COMPUTER VISION/Project/hitalert/res/test2.mp4"))
	if (!cap.open(0))
		return 0;

	Mat pic, oldPic, picRaw, diff;
	Mat oldGray, gray;
	vector<Point2f> prevPoints, nextPoints;
	vector<unsigned char> status;
	vector<float> err;

	cap >> pic;
	//resize(picRaw, pic, Size(picRaw.cols / 2, picRaw.rows / 2));
	oldPic = pic.clone();

	int angleRes = 60;
	vector<float> mean(angleRes);

	while (true) {
		int k = waitKey(1);
		if (k == ' ') {
			while (waitKey(1) != ' ');
		}
		else if (k == 27) {
			break;
		}

		cap >> picRaw;
		if (picRaw.empty())
			break;

		float roiScore[3];
		float nRoiScore[3];
		for (int i = 0; i < 3; i++) { // number of roi
			roiScore[i] = 0;
			nRoiScore[i] = 0;
		}

		pic = picRaw;
		//resize(picRaw, pic, Size(picRaw.cols / 2, picRaw.rows / 2));
		cvtColor(pic, gray, CV_BGR2GRAY);
		cvtColor(oldPic, oldGray, CV_BGR2GRAY);

		goodFeaturesToTrack(gray, prevPoints, 100, 0.01, 30);
		calcOpticalFlowPyrLK(oldGray, gray, prevPoints, nextPoints, status, err);

		Mat out;
		pic.copyTo(out);

		for (int i = 0; i < prevPoints.size(); i++) {
			if (!status[i])
				continue;

			float sumDiff = 0;
			float sumDist = 0;

			for (int j = 0; j < prevPoints.size(); j++) {
				if (status[j]) {
					float newDist = dist(nextPoints[i], nextPoints[j]);
					if (newDist < 100) {
						float oldDist =  dist(prevPoints[i], prevPoints[j]);
						float diffDist = newDist - oldDist;
						sumDiff += diffDist;
						sumDist += oldDist;
					}
				}
			}

			float score = sumDiff / sumDist;
			float scoreRadius = score * 300;

			if (score > 0) {
				circle(out, prevPoints[i], scoreRadius, Scalar(0, 0, 255), 2);
				for (int i = 0; i < 3; i++) { // number of roiScore regions
					if (roi[i].contains(prevPoints[i])) {
						roiScore[i] += score;
						nRoiScore[i]++;
					}
				}
			}

			
		}

		//flip(out, out, 0);
		//flip(out, out, 1);

//		for (int i = 0; i < 3; i++) { // number of roiScore regions
//			float roiScoreFinal = (int)((roiScore[i] / nRoiScore[i]) * 10000) / 10000.0;
//			rectangle(out, roi[i], Scalar(0, 255, 0), 2);
//			putText(out, to_string(roiScoreFinal), roi[i].tl() + Point(20, 70),
//				CV_FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 255), 2);
//		}
//
//		drawFps(out);
		imshow("opticalFlow", out);

		oldPic = pic.clone();
	}

	return 0;
}