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


int main(int argc, char** argv)
{
	VideoCapture cap;
	if (!cap.open("C:/Users/tae/Stuffs/Study/4_1_2110433 COMPUTER VISION/Project/hitalert/res/test2.mp4"))
	//if (!cap.open(0))
		return 0;

	Mat pic, oldPic, picRaw, diff;
	Mat oldGray, gray;
	vector<Point2f> prevPoints, nextPoints;
	vector<unsigned char> status;
	vector<float> err;

	cap >> picRaw;
	resize(picRaw, pic, Size(picRaw.cols / 2, picRaw.rows / 2));
	oldPic = pic.clone();

	int angleRes = 60;
	vector<float> mean(angleRes);

	while (waitKey(0) != 27) {
		cap >> picRaw;

		resize(picRaw, pic, Size(picRaw.cols / 2, picRaw.rows / 2));
		cvtColor(pic, gray, CV_BGR2GRAY);
		cvtColor(oldPic, oldGray, CV_BGR2GRAY);

		Point2f center(pic.cols / 2, pic.rows / 2);

		//absdiff(pic, oldPic, diff);
		//imshow("test", pic);
		//imshow("test2", diff);

		goodFeaturesToTrack(gray, prevPoints, 100, 0.01, 30);
		calcOpticalFlowPyrLK(oldGray, gray, prevPoints, nextPoints, status, err);

		Mat out;
		pic.copyTo(out);

		Mat distHist;
		circle(out, center, 8, Scalar(0, 0, 255), -1);

		for (int i = 0; i < angleRes; i++) {
			mean[i] = 0;
		}
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

						distHist.push_back(normalizedDiff);
						//line(out, nextPoints[i], nextPoints[j], normalizedDiff * Scalar(255, 255, 255), 3);
					}
				}
			}
			line(out, prevPoints[i], nextPoints[i], Scalar(0, 255, 255), 3);

			circle(out, center + (nextPoints[i] - prevPoints[i]), 2, Scalar(0, 255, 0), -1);
			//float angle = atan2(prevPoints[i].y - nextPoints[i].y, prevPoints[i].x - nextPoints[i].x) + CV_PI;
			//angle = angle * 180 / CV_PI;
			//mean[(int)(angle / (360 / angleRes))] += norm(nextPoints[i] - prevPoints[i]);
			//cout << (int)(angle / (360 / angleRes)) << " " << angle << " " << norm(nextPoints[i] - prevPoints[i]) << "\n";
		}


		flip(out, out, 0);
		flip(out, out, 1);

		//for (int i = 0; i < angleRes; i++) {
		//	float angle = i * 2 * CV_PI / angleRes;
		//	//cout << "!" << angle << " " << mean[i] << "\n";
		//	Point2f dir(cos(angle), sin(angle));
		//	line(out, center, center + mean[i] * dir,
		//		Scalar(255, 0, 255), 4);
		//}

//		drawFps(out);
		imshow("opticalFlow", out);

		oldPic = pic.clone();
	}

	return 0;
}