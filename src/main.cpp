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

void drawFps(Mat img) {
	static clock_t lastFrameTime;
	clock_t thisFrameTime = clock();
	float dt = (float)(thisFrameTime - lastFrameTime) / CLOCKS_PER_SEC;
	lastFrameTime = thisFrameTime;
	putText(img, to_string(1 / dt), Point(20, 40),
		CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 255));
}

int main(int argc, char** argv)
{
	VideoCapture cap;
	////if (!cap.open("C:/Users/tae/Stuffs/Study/4_1_2110433 COMPUTER VISION/Project/test/VID_20160830_072541.mp4"))
	if (!cap.open(1))
		return 0; 

	Mat pic, oldPic, picRaw, diff;
	Mat oldGray, gray;
	vector<Point2f> prevPoints, nextPoints;
	vector<unsigned char> status;
	vector<float> err;

	cap >> pic;
	//resize(picRaw, pic, Size(picRaw.cols / 2, picRaw.rows / 2));
	oldPic = pic.clone();

	while (waitKey(10) != 27) {
		cap >> pic;

		//resize(picRaw, pic, Size(picRaw.cols / 2, picRaw.rows / 2));
		cvtColor(pic, gray, CV_BGR2GRAY);
		cvtColor(oldPic, oldGray, CV_BGR2GRAY);

		//absdiff(pic, oldPic, diff);
		//imshow("test", pic);
		//imshow("test2", diff);
		
		goodFeaturesToTrack(gray, prevPoints, 100, 0.01, 30);
		calcOpticalFlowPyrLK(oldGray, gray, prevPoints, nextPoints, status, err);
		
		Mat out;
		pic.copyTo(out);
		for (int i = 0; i < prevPoints.size(); i++) {
			//circle(out, nextPoints[i], 4, Scalar(0, 255, 0), -1);
			if (!status[i]) {
				continue;
			}
			for (int j = 0; j < prevPoints.size(); j++) {
				if (status[j]) {
					float newDist = dist(nextPoints[i], nextPoints[j]);
					if (newDist < 100) {
						float distRatio = newDist / dist(prevPoints[i], prevPoints[j]);
						float logRatioRange = log2(2);
						float normalizedDiff = logRatioRange * log2(distRatio) + 0.5;
						circle(out, (nextPoints[i] + nextPoints[j]) / 2, 4,
							normalizedDiff * Scalar(255, 255, 255), -1);
						//line(out, nextPoints[i], nextPoints[j], normalizedDiff * Scalar(255, 255, 255), 3);
					}
				}
			}
			//line(out, prevPoints[i], nextPoints[i], Scalar(0, 255, 255), 3);
		}

		drawFps(out);

		imshow("opticalFlow", out);

		oldPic = pic.clone();
	}

	return 0;
}