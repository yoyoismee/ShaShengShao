#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <ctime>
#include <map>
#include <algorithm>

#include "HitAlert.hpp"
#include "Utils.hpp"
#include "Config.hpp"

using namespace cv;
using namespace std;


Mat yellow(Size(PROCESS_WIDTH, PROCESS_HEIGHT), CV_8UC3, Vec3b(0, 255, 255));
Mat red(Size(PROCESS_WIDTH, PROCESS_HEIGHT), CV_8UC3, Vec3b(0, 0, 255));

int gBlurSize = GS_BLUR_SIZE; // make it odd
int gBlurSigma = gBlurSize * GS_BLUR_SIGMA_REL;
float expBlurRatio = EXP_BLUR_RATIO;

int main(int argc, char** argv)
{
	VideoCapture cap;
	if (!cap.open(CAP_SRC))
		return 0;

	Rect roi = Rect(0, 0, ROI_WIDTH_REL * PROCESS_WIDTH, PROCESS_HEIGHT);
	vector<Rect> rois;
	int maxXShift = PROCESS_WIDTH * (1 - ROI_WIDTH_REL);
	for (int i = 0; i < ROI_COUNT; i++) {
		int xShift = maxXShift * i / (ROI_COUNT - 1);
		rois.push_back(roi + Point(xShift, 0));
	}


	HitAlert::Config haConfig;
	haConfig.frameHeight = PROCESS_HEIGHT;
	haConfig.frameWidth = PROCESS_WIDTH;
	haConfig.trackMaxCorners = TRACK_CORNERS;
	haConfig.trackQuality = TRACK_QUALITY;
	haConfig.trackMinDistance = TRACK_MIN_DIST;

	HitAlert hitAlert(haConfig);


	Mat pic, picRaw;
	Mat riskMapBlurred(PROCESS_HEIGHT, PROCESS_WIDTH, CV_8UC1);
	bool isPaused = false;
	bool isStepping = false; 

	while (true) {
		{
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

			if (isPaused)
				continue;
		}

		cap >> picRaw;
		if (picRaw.empty())
			break;
		resize(picRaw, pic, Size(PROCESS_WIDTH, PROCESS_HEIGHT));

		Mat discard;
		for (int i = 0; i < FRAMES_SKIP; i++)
			cap >> discard;

		hitAlert.setCurrentFrame(picRaw);
		hitAlert.calculateRiskMap();
		Mat riskMap = hitAlert.getRiskMap().clone();
		hitAlert.pushFrame();

		//// post processing
		GaussianBlur(riskMap, riskMap, Size(gBlurSize, gBlurSize), gBlurSigma);
		riskMapBlurred = riskMapBlurred * expBlurRatio + riskMap * (1 - expBlurRatio);

		Mat out = pic.clone();
		Mat riskMid, riskHi;
		threshold(riskMapBlurred, riskMid, 255 - TTC_MID, 255, THRESH_BINARY);
		threshold(riskMapBlurred, riskHi, 255 - TTC_LOW, 255, THRESH_BINARY);

		bitwise_or(out, yellow, out, riskMid - riskHi);
		bitwise_or(out, red, out, riskHi);

		Utils::drawFps(out);
		//imshow("risk", riskMap);
		//imshow("risk postprocessed", riskMapBlurred);
		imshow("OUT", out);
	}

	return 0;
}