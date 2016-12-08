#include "HitAlert.hpp"

HitAlert::HitAlert(Config config)
{
	config_ = config;
	Size frameSize(config_.frameWidth, config_.frameHeight);
	currentFrame_ = Mat(frameSize, CV_8UC1, Scalar(0));
	previousFrame_ = Mat(frameSize, CV_8UC1, Scalar(0));
	riskMap_ = Mat(frameSize, CV_8UC1, Scalar(0));
}



void HitAlert::setCurrentFrame(const Mat& newFrame)
{
	resize(newFrame, currentFrameRaw_, Size(config_.frameWidth, config_.frameHeight));

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
	previousFrameRaw_ = currentFrameRaw_.clone();
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
			trianglesRaw[i][4] > 0 &&
			nextPosition_[Point2f(trianglesRaw[i][0],trianglesRaw[i][1])].x > 0 &&
			nextPosition_[Point2f(trianglesRaw[i][2],trianglesRaw[i][3])].x > 0 &&
			nextPosition_[Point2f(trianglesRaw[i][4],trianglesRaw[i][5])].x > 0
			)
			triangles.push_back(trianglesRaw[i]);
	}

	vector<float> timeToCollides;
	Mat shape(previousFrame_.rows, previousFrame_.cols, CV_8UC1);
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
		
		// fix false alarm on lateral motions
		Size2f boundSizeBefore = getBoundingSize(pointsBefore);
		Size2f boundSizeAfter = getBoundingSize(pointsAfter);

		float heightRatio = boundSizeAfter.height / boundSizeBefore.height;
		float widthRatio = boundSizeAfter.width / boundSizeBefore.width;
		float shapeRatio = heightRatio / widthRatio;
		if (shapeRatio < 1)
			shapeRatio = 1 / shapeRatio;

		float areaBefore = contourArea(pointsBefore);
		float areaAfter = contourArea(pointsAfter);

		float lenBefore = sqrt(areaBefore);
		float lenAfter = sqrt(areaAfter);

		float ttc = lenBefore / (lenAfter - lenBefore);
		//float ttc = sqrt(areaAfter * areaBefore) / (areaAfter - areaBefore);
		timeToCollides.push_back(ttc);

		Scalar color = Scalar(ttc >= 0 ? (255 - ttc / shapeRatio) : 0);
		fillConvexPoly(riskMap_, pointsAfter, color);
		//fillConvexPoly(shape, pointsAfter, Scalar(128) * shapeRatio);
	}

#ifdef DEBUG
	// breakdown
	Mat trackedPoints = previousFrameRaw_.clone();
	Mat optFlow = previousFrameRaw_.clone();
	Mat delaunay = previousFrameRaw_.clone();
	Mat delaunayCompare = previousFrameRaw_.clone();
	for (int i = 0; i < previousTrackedPoints_.size(); i++) {
		circle(trackedPoints, previousTrackedPoints_[i], 4, Scalar(0, 255, 255), -1);

		circle(optFlow, currentTrackedPoints_[i], 4, Scalar(0, 0, 255), -1);
		line(optFlow, previousTrackedPoints_[i], currentTrackedPoints_[i], Scalar(0, 255, 255), 2);
	}
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

		line(delaunay, pointsBefore[0], pointsBefore[1], Scalar(255, 255, 0), 1);
		line(delaunay, pointsBefore[1], pointsBefore[2], Scalar(255, 255, 0), 1);
		line(delaunay, pointsBefore[2], pointsBefore[0], Scalar(255, 255, 0), 1);

		line(delaunayCompare, pointsBefore[0], pointsBefore[1], Scalar(255, 20, 0), 1);
		line(delaunayCompare, pointsBefore[1], pointsBefore[2], Scalar(255, 20, 0), 1);
		line(delaunayCompare, pointsBefore[2], pointsBefore[0], Scalar(255, 20, 0), 1);
		line(delaunayCompare, pointsBefore[0], pointsAfter[0], Scalar(0, 120, 255), 1);
		line(delaunayCompare, pointsBefore[1], pointsAfter[1], Scalar(0, 120, 255), 1);
		line(delaunayCompare, pointsBefore[2], pointsAfter[2], Scalar(0, 120, 255), 1);
		line(delaunayCompare, pointsAfter[0], pointsAfter[1], Scalar(255, 255, 0), 1);
		line(delaunayCompare, pointsAfter[1], pointsAfter[2], Scalar(255, 255, 0), 1);
		line(delaunayCompare, pointsAfter[2], pointsAfter[0], Scalar(255, 255, 0), 1);
		int areaBefore = (int)contourArea(pointsBefore);
		int areaAfter = (int)contourArea(pointsAfter);
		float ttc = sqrt(areaAfter * areaBefore) / (areaAfter - areaBefore);
		if (ttc < TTC_LOW && ttc >= 0) {
			line(delaunayCompare, pointsAfter[0], pointsAfter[1], Scalar(0, 0, 255), 2);
			line(delaunayCompare, pointsAfter[1], pointsAfter[2], Scalar(0, 0, 255), 2);
			line(delaunayCompare, pointsAfter[2], pointsAfter[0], Scalar(0, 0, 255), 2);
			Point center = (pointsAfter[0] + pointsAfter[1] + pointsAfter[2]) / 3;
			putText(delaunayCompare, to_string(ttc),
				center, CV_FONT_HERSHEY_PLAIN, 0.7, Scalar(0, 255, 255), 2);
			putText(delaunayCompare, to_string(areaAfter) + "x" + to_string(areaBefore),
				center + Point(0, 10), CV_FONT_HERSHEY_PLAIN, 0.7, Scalar(0, 255, 255), 1);
		}
	}

	imshow("Tracked Points", trackedPoints);
	imshow("Optical Flow", optFlow);
	imshow("Delaunay Triangulation", delaunay);
	imshow("Delaunay Before/After", delaunayCompare);
	imshow("r", riskMap_);
	//imshow("shape", shape);
#endif 
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



Size2f HitAlert::getBoundingSize(vector<Point> points)
{
	int minX = points[0].x;
	int minY = points[0].y;
	int maxX = points[0].x;
	int maxY = points[0].y;

	for (int i = 0; i < points.size(); i++) {
		if (points[i].x < minX) 
			minX = points[i].x;
		if (points[i].y < minY)
			minY = points[i].y;
		if (points[i].x > maxX)
			minX = points[i].x;
		if (points[i].y > maxY)
			minY = points[i].y;
	}

	return Size2f(maxX - minX, maxY - minY);
}