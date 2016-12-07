#pragma once

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

using namespace std;
using namespace cv;

class HitAlert {
public:
	struct Config {
		int frameWidth;
		int frameHeight;
		int trackMaxCorners;
		float trackQuality;
		int trackMinDistance;
	};

	HitAlert(Config config);

	void setCurrentFrame(const Mat& newFrame);
	void pushFrame();

	void calculateRiskMap();
	map<Point2f, Point2f, Utils::cmpPoint2f> getTrackedPoints();
	//Size getFrameSize();
	Mat getPreviousFrame();
	Mat getCurrentFrame();
	Mat getRiskMap();

private:
	struct Config config_;

	Mat previousFrame_;	// grayscale
	Mat currentFrame_;	// grayscale
	Mat riskMap_;

	vector<Point2f> previousTrackedPoints_;
	vector<Point2f> currentTrackedPoints_;
	std::map<Point2f, Point2f, Utils::cmpPoint2f> nextPosition_;

	Size2f getBoundingSize(vector<Point> points);
};