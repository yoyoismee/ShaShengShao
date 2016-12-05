#include "HitAlert.hpp"

HitAlert::HitAlert(Config config)
{
	config_ = config;
	currentFrame_ = Mat(config_.frameHeight, config_.frameWidth, CV_8UC1);
	previousFrame_ = Mat(config_.frameHeight, config_.frameWidth, CV_8UC1);
	riskMap_ = Mat(config_.frameHeight, config_.frameWidth, CV_8UC1);
}



void HitAlert::setCurrentFrame(const Mat& newFrame)
{
	Mat newFrameGray;
	cvtColor(newFrame, newFrameGray, CV_BGR2GRAY);
	resize(newFrameGray, currentFrame_, Size(config_.frameWidth, config_.frameHeight));

	// TODO Preprocess?

	////Mat sharpen = (
	////	Mat_<uchar>(3, 3) <<
	////	-1, -1, -1,
	////	-1, 9, -1,
	////	-1, -1, -1
	////	);
	////filter2D(pic, pic, -1, sharpen);
}



void HitAlert::pushFrame()
{
	previousFrame_ = currentFrame_.clone();
}



void HitAlert::calculateRiskMap()
{
	riskMap_ = 0;
	nextPosition_.clear();

	goodFeaturesToTrack(
		previousFrame_, previousTrackedPoints_, 
		config_.trackMaxCorners, 
		config_.trackQuality,
		config_.trackMinDistance);
	if (previousTrackedPoints_.size() == 0) return;

	vector<unsigned char> status;
	vector<float> err;
	calcOpticalFlowPyrLK(
		previousFrame_, currentFrame_, 
		previousTrackedPoints_, currentTrackedPoints_,
		status, err);


	// delaunay
	Subdiv2D subdiv(Rect(0, 0, config_.frameWidth, config_.frameHeight));
	for (int i = 0; i < previousTrackedPoints_.size(); i++) {
		if (status[i]) {
			subdiv.insert(previousTrackedPoints_[i]);
			nextPosition_[previousTrackedPoints_[i]] = currentTrackedPoints_[i];
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

	vector<float> timeToCollides;
	for (int i = 0; i < triangles.size(); i++) {
		Vec6f t = triangles[i];
		vector<Point> pointsBefore;
		pointsBefore.push_back(Point(t[0], t[1]));
		pointsBefore.push_back(Point(t[2], t[3]));
		pointsBefore.push_back(Point(t[4], t[5]));

		vector<Point> pointsAfter;
		pointsAfter.push_back(nextPosition_[pointsBefore[0]]);
		pointsAfter.push_back(nextPosition_[pointsBefore[1]]);
		pointsAfter.push_back(nextPosition_[pointsBefore[2]]);

		float areaBefore = contourArea(pointsBefore);
		float areaAfter = contourArea(pointsAfter);

		float lenBefore = sqrt(areaBefore);
		float lenAfter = sqrt(areaAfter);

		float ttc = lenBefore / (lenAfter - lenBefore);
		timeToCollides.push_back(ttc);

		Scalar color = Scalar(ttc >= 0 ? (255 - ttc) : 0);
		fillConvexPoly(riskMap_, pointsAfter, color);
	}
}



map<Point2f, Point2f, Utils::cmpPoint2f> HitAlert::getTrackedPoints()
{
	return nextPosition_;
}



Mat HitAlert::getPreviousFrame()
{
	return previousFrame_;
}



Mat HitAlert::getCurrentFrame()
{
	return currentFrame_;
}



Mat HitAlert::getRiskMap()
{
	return riskMap_;
}