#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <ctime>
#include <map>

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
	if (!cap.open("C:/Users/tae/Stuffs/Study/4_1_2110433 COMPUTER VISION/Project/hitalert/res/test2.mp4"))
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

	int angleRes = 60;
	vector<float> mean(angleRes);

	bool isPaused = false;
	bool isStepping = false;

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
		cvtColor(pic, gray, CV_BGR2GRAY);
		cvtColor(oldPic, oldGray, CV_BGR2GRAY);

		goodFeaturesToTrack(gray, prevPoints, 100, 0.01, 30);
		calcOpticalFlowPyrLK(oldGray, gray, prevPoints, nextPoints, status, err);

		Mat out;
		pic.copyTo(out);

		vector<float> roiScore(ROI_COUNT);
		vector<float> nRoiScore(ROI_COUNT);
		for (int i = 0; i < rois.size(); i++) {
			roiScore[i] = 0;
			nRoiScore[i] = 0;
		}

		for (int i = 0; i < prevPoints.size(); i++) {
			if (!status[i])
				continue;

			float sumDiff = 0;
			float sumDist = 0;

			for (int j = 0; j < prevPoints.size(); j++) {
				if (status[j]) {
					float newDist = norm(nextPoints[i] - nextPoints[j]);
					if (newDist < 100) {
						float oldDist =  norm(prevPoints[i] - prevPoints[j]);
						float diffDist = newDist - oldDist;
						sumDiff += diffDist;
						sumDist += oldDist;
					}
				}
			}

			float score = sumDist > 0 ? sumDiff / sumDist : 0;
			float scoreRadius = score * 300;

			circle(out, prevPoints[i], 3, Scalar(0, 255, 255), CV_FILLED);
			if (score >= 0) {
				circle(out, prevPoints[i], scoreRadius, Scalar(0, 0, 255), 2);
				for (int j = 0; j < ROI_COUNT; j++) {
					if (rois[j].contains(prevPoints[i])) {
						roiScore[j] += score;
						nRoiScore[j]++;
					}
				}
			}
		}

		for (int i = 0; i < ROI_COUNT; i++) { // number of roiScore regions
			float roiScoreFinal = nRoiScore[i] > 0 ? (int)((roiScore[i] / nRoiScore[i]) * 1000) : 0;
			rectangle(out, rois[i], Scalar(0, 255, 0), 2);
			putText(out, to_string(roiScoreFinal), rois[i].tl() + Point(20, 70), 
				CV_FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 255), 2);

			Point tl = rois[i].tl() + Point(20, 70);
			Point br = tl + Point(50, roiScoreFinal * 3);
			rectangle(out, Rect(tl, br), Scalar(0, 255, 255), CV_FILLED);
		}




		std::map<Point2f, Point2f, cmpPoint2f> nextPosition;

		// delaunay
		Subdiv2D subdiv(Rect(0, 0, PROCESS_WIDTH, PROCESS_HEIGHT));
		for (int i = 0; i < prevPoints.size(); i++) {
			if (status[i]) {
				subdiv.insert(prevPoints[i]);
				nextPosition[prevPoints[i]] = nextPoints[i];
			}
		}
		//vector<vector<Point2f>> delauneyNeighbor;
		//for (int i = 0; i < prevPoints.size(); i++) {
		//	delauneyNeighbor.push_back(vector<Point2f>());
		//}

		vector<Vec6f> triangles;
		subdiv.getTriangleList(triangles);

		vector<float> areaRatio;
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
			areaRatio.push_back(areaAfter / areaBefore);
			//cout << areaRatio[i] <<"\n";
		}
		//cout << "\n" << "\n" << "\n";

		Mat outOverlay = out.clone();
		Scalar white(128, 128, 128);
		for (int i = 0; i < triangles.size(); i++) {
			Vec6f t = triangles[i];
			vector<Point> points;
			points.push_back(Point(t[0], t[1]));
			points.push_back(Point(t[2], t[3]));
			points.push_back(Point(t[4], t[5]));

			float s = areaRatio[i] * areaRatio[i] * areaRatio[i];
			Scalar color(0, 255 * (2 - s), 255 * s);
			//Scalar color = Scalar(128, 128, 128) * s;
			
			fillConvexPoly(outOverlay, points, color);

			//line(outOverlay, points[0], points[1], Scalar(0, 255, 0), 2);
			//line(outOverlay, points[1], points[1], Scalar(0, 255, 0), 2);
			//line(outOverlay, points[2], points[0], Scalar(0, 255, 0), 2);
		}





		Utils::drawFps(out);

		imshow("opticalFlow", out * 0.7 + outOverlay * 0.3);
		imshow("color", outOverlay);

		oldPic = pic.clone();
	}

	return 0;
}