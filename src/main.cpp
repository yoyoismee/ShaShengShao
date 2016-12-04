#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <ctime>
#include <map>
#include <algorithm>

#include "Utils.hpp"
#include "Config.hpp"

using namespace cv;
using namespace std;

struct cmpPoint2f {
	bool operator()(const Point2f& a, const Point2f& b) const {
		return a.x != b.x ? a.x < b.x : a.y < b.y;
	}
};

int main(int argc, char** argv)
{
	Rect roi = Rect(0, 0, ROI_WIDTH * PROCESS_WIDTH, PROCESS_HEIGHT);
	vector<Rect> rois;
	int maxXShift = PROCESS_WIDTH * (1 - ROI_WIDTH);
	for (int i = 0; i < ROI_COUNT; i++) {
		int xShift = maxXShift * i / (ROI_COUNT - 1);
		rois.push_back(roi + Point(xShift, 0));
	}

	VideoCapture cap;
	if (!cap.open(CAP_SRC))
		//if (!cap.open(0))
		return 0;

	Mat pic, oldPic, picRaw, diff;
	Mat oldGray, gray;
	vector<Point2f> prevPoints, nextPoints;
	vector<unsigned char> status;
	vector<float> err;


	cap >> picRaw;
	resize(picRaw, pic, Size(PROCESS_WIDTH, PROCESS_HEIGHT));
	oldPic = pic.clone();

	bool isPaused = false;
	bool isStepping = false;

	Mat outTtcBlurred(PROCESS_HEIGHT, PROCESS_WIDTH, CV_8UC3);

	while (true) {
		if (isStepping)
			isPaused = true;

		int k = waitKey(1);
		if (k == ' ') {
			isPaused = !isPaused;
			isStepping = false;
		}
		else if (k == 27)
			break;
		else if (k == 's') {
			isStepping = true;
			isPaused = false;
		}

		if (isPaused) {
			continue;
		}

		cap >> picRaw;
		if (picRaw.empty())
			break;

		Mat discard;
		for (int i = 0; i < FRAMES_SKIP; i++) {
			cap >> discard;
		}

		pic = picRaw;
		resize(picRaw, pic, Size(PROCESS_WIDTH, PROCESS_HEIGHT));

		//Mat sharpen = (
		//	Mat_<uchar>(3, 3) <<
		//	-1, -1, -1,
		//	-1, 9, -1,
		//	-1, -1, -1
		//	);
		//filter2D(pic, pic, -1, sharpen);

		cvtColor(pic, gray, CV_BGR2GRAY);
		cvtColor(oldPic, oldGray, CV_BGR2GRAY);

		goodFeaturesToTrack(gray, prevPoints, 100, 0.01, 20);
		calcOpticalFlowPyrLK(oldGray, gray, prevPoints, nextPoints, status, err);

		Mat out;
		pic.copyTo(out);

		std::map<Point2f, Point2f, cmpPoint2f> nextPosition;

		// delaunay
		Subdiv2D subdiv(Rect(0, 0, PROCESS_WIDTH, PROCESS_HEIGHT));
		for (int i = 0; i < prevPoints.size(); i++) {
			if (status[i]) {
				subdiv.insert(prevPoints[i]);
				nextPosition[prevPoints[i]] = nextPoints[i];

				// draw optical flow
				//circle(out, prevPoints[i], 5, Scalar(255, 0, 0), -1);
				//circle(out, nextPoints[i], 5, Scalar(255, 255, 0), -1);
				//line(out, prevPoints[i], nextPoints[i], Scalar(0, 255, 0), 2);
			}
		}

		vector<Vec6f> trianglesRaw, triangles;
		subdiv.getTriangleList(trianglesRaw);
		for (int i = 0; i < trianglesRaw.size(); i++) {
			if (
				trianglesRaw[i][0] > 0 &&
				trianglesRaw[i][2] > 0 &&
				trianglesRaw[i][4] > 0
				)
				triangles.push_back(trianglesRaw[i]);
		}

		Mat outTtcNew(out.rows, out.cols, CV_8UC3);

		vector<float> timeToCollides;
		for (int i = 0; i < triangles.size(); i++) {
			Vec6f t = triangles[i];
			vector<Point2f> pointsBefore;
			pointsBefore.push_back(Point2f(t[0], t[1]));
			pointsBefore.push_back(Point2f(t[2], t[3]));
			pointsBefore.push_back(Point2f(t[4], t[5]));

			vector<Point2f> pointsAfter;
			pointsAfter.push_back(nextPosition[pointsBefore[0]]);
			pointsAfter.push_back(nextPosition[pointsBefore[1]]);
			pointsAfter.push_back(nextPosition[pointsBefore[2]]);

			float areaBefore = contourArea(pointsBefore);
			float areaAfter = contourArea(pointsAfter);
			
			float lenBefore = sqrt(areaBefore);
			float lenAfter = sqrt(areaAfter);

			float ttc = lenBefore / (lenAfter - lenBefore);
			timeToCollides.push_back(ttc);
		
			vector<Point> points;
			points.push_back(Point(t[0], t[1]));
			points.push_back(Point(t[2], t[3]));
			points.push_back(Point(t[4], t[5]));

			float s = timeToCollides[i];
			Scalar color = Scalar(128, 128, 128) * (1 + 10 / s);
			fillConvexPoly(outTtcNew, points, color);
		
			//line(out, points[0], points[1], Scalar(255, 0, 0), 1);
			//line(out, points[1], points[2], Scalar(255, 0, 0), 1);
			//line(out, points[2], points[0], Scalar(255, 0, 0), 1);

			//line(out, nextPosition[points[0]], nextPosition[points[1]], Scalar(255, 0, 0), 1);
			//line(out, nextPosition[points[1]], nextPosition[points[2]], Scalar(255, 0, 0), 1);
			//line(out, nextPosition[points[2]], nextPosition[points[0]], Scalar(255, 0, 0), 1);
		}

		// post processing
		GaussianBlur(outTtcNew, outTtcNew, Size(25, 25), 18);
		threshold(outTtcNew, outTtcNew, 160, 255, THRESH_BINARY);
		outTtcBlurred = outTtcBlurred * 0.9 + outTtcNew * 0.1;


		Utils::drawFps(out);

		Mat yellow(Size(out.cols, out.rows), CV_8UC3, Vec3b(0, 255, 255));
		Mat red(Size(out.cols, out.rows), CV_8UC3, Vec3b(0, 0, 255));

		Mat outFinHi, outFinMid;
		threshold(outTtcBlurred, outFinMid, 130, 255, THRESH_BINARY);
		threshold(outTtcBlurred, outFinHi, 160, 255, THRESH_BINARY);

		bitwise_and(outFinMid - outFinHi, yellow, outFinMid);
		bitwise_and(outFinHi, red, outFinHi);

		imshow("opticalFlow", out + outFinMid + outFinHi);
		imshow("TTC", outTtcBlurred);
		imshow("OUT", outFinMid + outFinHi);

		oldPic = pic.clone();
	}

	return 0;
}