#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <ctime>
#include <map>
#include <algorithm>
#include <memory>
#include <Windows.h>
#include <mmsystem.h>

#include "HitAlert.hpp"
#include "Utils.hpp"
#include "Config.hpp"

#include "Pundlik13.hpp"

using namespace cv;
using namespace std;


void drawOutput(Mat &out, const Mat riskHi, const Mat riskMid);
void playAlertSound(const Mat riskHi, const Mat riskMid);


Mat yellow(Size(PROCESS_WIDTH, PROCESS_HEIGHT), CV_8UC3, Vec3b(0, 255, 255));
Mat red(Size(PROCESS_WIDTH, PROCESS_HEIGHT), CV_8UC3, Vec3b(0, 0, 255));

int gBlurSize = GS_BLUR_SIZE;
int gBlurSigma = gBlurSize * GS_BLUR_SIGMA_REL;
float expBlurRatio = EXP_BLUR_RATIO;

int main(int argc, char* argv[])
{
	VideoCapture cap;
	if (argc >= 2) {
		string capSource(argv[1]);
		if (!cap.open(capSource)) {
			cout << "Error: Cannot open VideoCapture from " << capSource << ".\n";
			return 0;
		}
	} 
	else if (!cap.open(CAP_SRC)) {
		cout << "Error: Cannot open VideoCapture from " << CAP_SRC << ".\n";
		return 0;
	}

#ifdef OUTPUT_VIDEO
	VideoWriter vWriter;
	int format = VideoWriter::fourcc('A', 'V', 'C', '1');
	if (!vWriter.open(OUTPUT_VIDEO_PATH, format, 30, Size(PROCESS_WIDTH, PROCESS_HEIGHT)))
		return 0;
#endif


	HitAlert::Config haConfig;
	haConfig.frameHeight = PROCESS_HEIGHT;
	haConfig.frameWidth = PROCESS_WIDTH;
	haConfig.trackMaxCorners = TRACK_CORNERS;
	haConfig.trackQuality = TRACK_QUALITY;
	haConfig.trackMinDistance = TRACK_MIN_DIST;

	HitAlert hitAlert(haConfig);


#ifdef COMPARE
	Pundlik13::Config plConfig;
	plConfig.frameHeight = PROCESS_HEIGHT;
	plConfig.frameWidth = PROCESS_WIDTH;
	plConfig.trackMaxCorners = TRACK_CORNERS;
	plConfig.trackQuality = TRACK_QUALITY;
	plConfig.trackMinDistance = TRACK_MIN_DIST;

	Pundlik13 pundlik(plConfig);
#endif


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

		////////////////////////////// risk map calculation //////////////////////////////

		hitAlert.setCurrentFrame(picRaw);
		hitAlert.calculateRiskMap();
		Mat riskMap = hitAlert.getRiskMap().clone();
		hitAlert.pushFrame();

#ifdef COMPARE
		pundlik.setCurrentFrame(picRaw);
		pundlik.calculateRiskMap();
		pundlik.pushFrame();
#endif

		////////////////////////////// post-processing //////////////////////////////

		// smoothing
		GaussianBlur(riskMap, riskMap, Size(gBlurSize, gBlurSize), gBlurSigma);
		riskMapBlurred = riskMapBlurred * expBlurRatio + riskMap * (1 - expBlurRatio);

		// thresholding
		Mat riskMid, riskHi;
		threshold(riskMapBlurred, riskMid, 255 - TTC_MID, 255, THRESH_BINARY);
		threshold(riskMapBlurred, riskHi, 255 - TTC_LOW, 255, THRESH_BINARY);

		// ---------- draw output image  ---------- 
		Mat out = pic.clone();
		drawOutput(out, riskHi, riskMid);
		
		// alert
		playAlertSound(riskHi, riskMid);

		Utils::drawFps(out);
#ifdef DEBUG
		imshow("risk postprocessed", riskMapBlurred);
#endif
		imshow("OUT", out);

#ifdef OUTPUT_VIDEO
		vWriter << out;
#endif
	}

	return 0;
}



void drawOutput(Mat &out, const Mat riskHi, const Mat riskMid)
{
	// make risk area on output image red/yellow
	bitwise_or(out, yellow, out, riskMid - riskHi);
	bitwise_or(out, red, out, riskHi);

	// center line
	line(out, Point(out.cols / 2, 0), Point(out.cols / 2, out.rows), Scalar(255, 128, 0), 2);

	//centroid of high risk areas
	Moments m = moments(riskHi, true);
	Point riskHiCentroid(m.m10 / m.m00, m.m01 / m.m00);
	circle(out, riskHiCentroid, 10, Scalar(255, 0, 255), 2);

	// bounding lines
	line(out, Point(out.cols * (1 + ROI_WIDTH_REL) / 2, 0), Point(out.cols * (1 + ROI_WIDTH_REL) / 2, out.rows), Scalar(255, 255, 0), 2);
	line(out, Point(out.cols * (1 - ROI_WIDTH_REL) / 2, 0), Point(out.cols * (1 - ROI_WIDTH_REL) / 2, out.rows), Scalar(255, 255, 0), 2);
}



void playAlertSound(const Mat riskHi, const Mat riskMid)
{
	// calculate for play sound
#ifdef DEBUG
	imshow("riskHi", riskHi);
	imshow("riskMid", riskMid);
#endif
	int centerRoiWidth = PROCESS_WIDTH * ROI_WIDTH_REL;
	int sideRoiWidth = (PROCESS_WIDTH - centerRoiWidth) / 2;
	Rect centerRoi((PROCESS_WIDTH - centerRoiWidth) / 2, 0, centerRoiWidth, PROCESS_HEIGHT);
	Rect leftRoi(0, 0, sideRoiWidth, PROCESS_HEIGHT);
	Rect rightRoi((PROCESS_WIDTH + centerRoiWidth) / 2, 0, sideRoiWidth, PROCESS_HEIGHT);

	Mat riskHiClone = riskHi.clone();
	Mat riskMidClone = riskMid.clone();
	if (sum(riskHi)[0] > 0) {
		// alert Highrisk
		double midHighRisk = sum(riskHi(centerRoi))[0];
		double leftHighRisk = sum(riskHi(leftRoi))[0];
		double rightHighRisk = sum(riskHi(rightRoi))[0];
		if (midHighRisk > 0) {
			PlaySound(TEXT(HITALERT_BASE_PATH "/res/MidHigh.wav"), NULL, SND_ASYNC);
		}
#ifdef ALERT_LEFT_RIGHT
		// alert left and right
		else if (leftHighRisk > 0) {
			PlaySound(TEXT(HITALERT_BASE_PATH "/res/MidHigh.wav"), NULL, SND_ASYNC);
		}
		else if (rightHighRisk > 0) {
			PlaySound(TEXT(HITALERT_BASE_PATH "/res/MidHigh.wav"), NULL, SND_ASYNC);
		}
#endif
	}
	else if (sum(riskMid)[0] > 0) {
		// alert lowRisk
		double midLowRisk = sum(riskMid(centerRoi))[0];
		double leftLowRisk = sum(riskMid(leftRoi))[0];
		double rightLowRisk = sum(riskMid(rightRoi))[0];
#ifndef ALERT_HIGH_ONLY
		if (midLowRisk > 0) {
			PlaySound(TEXT(HITALERT_BASE_PATH "/res/MidHigh.wav"), NULL, SND_ASYNC);
		}
#ifdef ALERT_LEFT_RIGHT
		// alert left and right
		else if (leftLowRisk > 0) {
			PlaySound(TEXT(HITALERT_BASE_PATH "/res/MidHigh.wav"), NULL, SND_ASYNC);
		}
		else if (rightLowRisk > 0) {
			PlaySound(TEXT(HITALERT_BASE_PATH "/res/MidHigh.wav"), NULL, SND_ASYNC);
		}
#endif
#endif
	}
	else {
		PlaySound(NULL, 0, 0);
	}
}
