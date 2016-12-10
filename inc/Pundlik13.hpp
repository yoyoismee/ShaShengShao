#pragma once

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <ctime>
#include <map>
#include <algorithm>
#include <set>

#include "Utils.hpp"
#include "Config.hpp"

using namespace std;
using namespace cv;

class Pundlik13 {
public:
	struct Config {
		int frameWidth;
		int frameHeight;
		int trackMaxCorners;
		float trackQuality;
		int trackMinDistance;
	};

	Pundlik13(Config config);

	void setCurrentFrame(const Mat& newFrame);
	void pushFrame();

	void calculateRiskMap();
	map<Point, Point, Utils::cmpPoint> getTrackedPoints();
	//Size getFrameSize();
	Mat getPreviousFrame();
	Mat getCurrentFrame();
	Mat getRiskMap();

private:
	struct Config config_;

	Mat previousFrameRaw_;
	Mat currentFrameRaw_;
	Mat previousFrame_;	// grayscale
	Mat currentFrame_;	// grayscale
	Mat riskMap_;

	vector<Point2f> previousTrackedPoints_;
	vector<Point2f> currentTrackedPoints_;
	std::map<Point, Point, Utils::cmpPoint> nextPosition_;
	std::map<Point, std::set<Point, Utils::cmpPoint>, Utils::cmpPoint> neighbor_;
};