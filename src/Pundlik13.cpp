#include "Pundlik13.hpp"

Pundlik13::Pundlik13(Config config)
{
	config_ = config;
	Size frameSize(config_.frameWidth, config_.frameHeight);
	currentFrame_ = Mat(frameSize, CV_8UC1, Scalar(0));
	previousFrame_ = Mat(frameSize, CV_8UC1, Scalar(0));
	riskMap_ = Mat(frameSize, CV_8UC1, Scalar(0));
}



void Pundlik13::setCurrentFrame(const Mat& newFrame)
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



void Pundlik13::pushFrame()
{
	previousFrame_ = currentFrame_.clone();
	previousFrameRaw_ = currentFrameRaw_.clone();
}



void Pundlik13::calculateRiskMap()
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
			Point p(previousTrackedPoints_[i].x, previousTrackedPoints_[i].y);
			nextPosition_[p] = currentTrackedPoints_[i];
		}
	}

	vector<Vec6f> trianglesRaw, triangles;
	subdiv.getTriangleList(trianglesRaw);
	for (int i = 0; i < trianglesRaw.size(); i++) {
		if (
			trianglesRaw[i][0] > 0 &&
			trianglesRaw[i][2] > 0 &&
			trianglesRaw[i][4] > 0 &&
			nextPosition_[Point(trianglesRaw[i][0], trianglesRaw[i][1])].x > 0 &&
			nextPosition_[Point(trianglesRaw[i][2], trianglesRaw[i][3])].x > 0 &&
			nextPosition_[Point(trianglesRaw[i][4], trianglesRaw[i][5])].x > 0
			)
			triangles.push_back(trianglesRaw[i]);
	}

	neighbor_.clear();
	vector<vector<Point>> trianglesBefore, trianglesAfter;
	for (int i = 0; i < triangles.size(); i++) {
		Vec6f t = triangles[i];
		vector<Point> pointsBefore;
		pointsBefore.push_back(Point(t[0], t[1]));
		pointsBefore.push_back(Point(t[2], t[3]));
		pointsBefore.push_back(Point(t[4], t[5]));

		neighbor_[pointsBefore[0]].insert(pointsBefore[1]);
		neighbor_[pointsBefore[0]].insert(pointsBefore[2]);
		neighbor_[pointsBefore[1]].insert(pointsBefore[0]);
		neighbor_[pointsBefore[1]].insert(pointsBefore[2]);
		neighbor_[pointsBefore[2]].insert(pointsBefore[1]);
		neighbor_[pointsBefore[2]].insert(pointsBefore[0]);
	}

	map<Point, float, Utils::cmpPoint> ttc;
	map<Point, Point, Utils::cmpPoint> collisionPoint;
	for (map<Point, std::set<Point, Utils::cmpPoint>, Utils::cmpPoint>::iterator i = neighbor_.begin(); i != neighbor_.end(); i++) {
		set<Point, Utils::cmpPoint> neighbors = i -> second;
		Point pi = i->first;
		Point qi = nextPosition_[pi];
		float sumPQ = 0, sumP = 0;

		Point lateralMotion = qi - pi;
		for (set<Point, Utils::cmpPoint>::iterator j = neighbors.begin(); j != neighbors.end(); j++) {
			Point pk = *j;
			Point qk = nextPosition_[pk];
			sumPQ += norm(qi - qk) - norm(pi - pk);
			sumP += norm(pi - pk);

			lateralMotion += qk - pk;
		}

		float expansion = sumPQ / sumP;
		ttc[pi] = 1 / expansion;

		lateralMotion /= (int)neighbors.size();
		collisionPoint[pi] = lateralMotion * ttc[pi];
	}

	Mat risks = currentFrameRaw_.clone();
	for (map<Point, float, Utils::cmpPoint>::iterator i = ttc.begin(); i != ttc.end(); i++) {
		Point p = i->first;
		if (100 - i->second > 0 && i->second >= 0) {
			cout << i->second << "\n";
			circle(risks, p, 100 - i->second, Scalar(0, 0, 255), 2);
			circle(risks, p, 3, Scalar(255, 0, 0), -1);
		}
	}
	imshow("Pundlik13", risks);
}



map<Point, Point, Utils::cmpPoint> Pundlik13::getTrackedPoints()
{
	return nextPosition_;
}



Mat Pundlik13::getPreviousFrame()
{
	return previousFrame_;
}



Mat Pundlik13::getCurrentFrame()
{
	return currentFrame_;
}



Mat Pundlik13::getRiskMap()
{
	return riskMap_;
}