#include <iostream>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <algorithm>

#include "Utils.hpp"

using namespace cv;
using namespace std;


float dist(Point2f p1, Point2f p2) {
    float dx = (p1.x - p2.x);
    float dy = (p1.y - p2.y);
    return sqrt(dx * dx + dy * dy);
}

Point2f camflow (vector<Point2f> prev,vector<Point2f> next,vector<unsigned char> sta,double trim=0.2){
    vector<double> x;
    vector<double> y;
    for(int i=0;i<prev.size();i++){
        if(sta[i]){
            x.push_back(prev[i].x-next[i].x);
            y.push_back(prev[i].y-next[i].y);

        }
    }
    double retx=0;
    double rety=0;
    sort(x.begin(),x.end());
    sort(y.begin(),y.end());
    for(int i = (int)x.size()*(trim/2);i<x.size()*(1-trim/2);i++){
        retx+=x[i];
        rety+=y[i];
    }

    return  Point2f(retx/x.size()*(1-trim),rety/y.size()*(1-trim));
}



int main() {
    VideoCapture cap("/Users/mac/Desktop/ShaShengShao/res/testsubject.mp4");
    Mat in;

    Mat pic, oldPic, picRaw, diff;
    Mat oldGray, gray;
    vector<Point2f> prevPoints, nextPoints;
    vector<unsigned char> status;
    vector<float> err;

    cap >> pic;
    //resize(picRaw, pic, Size(picRaw.cols / 2, picRaw.rows / 2));
    oldPic = pic.clone();

    while (1) {
        cap >> pic;

        cvtColor(pic, gray, CV_BGR2GRAY);
        cvtColor(oldPic, oldGray, CV_BGR2GRAY);

        //absdiff(pic, oldPic, diff);
        //imshow("test", pic);
        //imshow("test2", diff);

        goodFeaturesToTrack(gray, prevPoints, 100, 0.01, 30);
        calcOpticalFlowPyrLK(oldGray, gray, prevPoints, nextPoints, status, err);
        Point2f camF = camflow(prevPoints,nextPoints,status,0.3);
        Mat out;
        pic.copyTo(out);
        camF*=1.5;  ///<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<fucking adjust this value!!!!!
        for (int i = 0; i < prevPoints.size(); i++) {
           // circle(out, nextPoints[i], 4, Scalar(0, 255, 0), -1);
            if (!status[i]) {
                continue;
            }
          /*  for (int j = 0; j < prevPoints.size(); j++) {
                if (status[j]) {
                    float newDist = dist(nextPoints[i], nextPoints[j]);
                    if (newDist < 100) {
                        float distRatio = newDist / dist(prevPoints[i], prevPoints[j]);
                        float logRatioRange = log2(2);
                        float normalizedDiff = logRatioRange * log2(distRatio) + 0.5;
                        circle(out, (nextPoints[i] + nextPoints[j]) / 2, 4,
                               normalizedDiff * Scalar(255, 255, 255), -1);
                        line(out, nextPoints[i], nextPoints[j], normalizedDiff * Scalar(255, 255, 255), 3);
                    }
                }
            }*/
            if(dist(prevPoints[i],nextPoints[i]-camF)>20) {
                circle(out, prevPoints[i], 4, Scalar(255, 0, 255), -1);
                line(out, prevPoints[i], nextPoints[i], Scalar(255, 0, 0), 2);
                line(out, prevPoints[i],nextPoints[i] + camF,
                     Scalar(0, 0, 255), 2);
            }else {
                circle(out, prevPoints[i], 4, Scalar(100, 100, 100), -1);
                line(out, prevPoints[i], nextPoints[i], Scalar(100, 0, 0), 2);
                line(out, prevPoints[i],nextPoints[i] + camF, Scalar(0, 0, 100), 2);

            }
        }

        Utils::drawFps(out);
        circle(out,Point2f(200,200),3,Scalar(255,0,255),-1);
        line(out, Point2f(200,200), Point2f(200,200)+camF, Scalar(180, 255, 50), 2);


        imshow("opticalFlow", out);
        Mat diff = pic-oldPic;
        Mat big(pic.rows * 2, pic.cols*2, CV_8UC3,Scalar(255,255,255));
        big(Rect((int)(pic.cols*0.5),(int)(pic.rows*0.5),pic.cols,pic.rows))=0;
        big(Rect((int)(pic.cols*0.5),(int)(pic.rows*0.5),pic.cols,pic.rows))+=pic;
        big(Rect((int)((pic.cols*0.5)-camF.x),(int)((pic.rows*0.5)-camF.y),pic.cols,pic.rows))-=oldPic;

        imshow("diff",big);
        imshow("normaldiff",diff);
        big = big(Rect((int)(pic.cols*0.5),(int)(pic.rows*0.5),pic.cols,pic.rows));
        imshow("croped",big);

        oldPic = pic.clone();

        if(waitKey(0) == 27) break;
    }

    return 0;
}
