#ifndef my_server_CUDAOPTICALFLOW_HPP_
#define my_server_CUDAOPTICALFLOW_HPP_

#include <iostream>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "PID.hpp"
#include <ctime>

using namespace cv;
using namespace cv::gpu;
using namespace std;


#define xMax 640
#define yMax 480
#define centerX xMax / 2
#define centerY yMax / 2
#define rPointY 340

#define FRONTX1L 308
#define FRONTY1L 200

#define FRONTX1R 394
#define FRONTY1R 200

#define FRONTX2L 208
#define FRONTY2L 479

#define FRONTX2R 482
#define FRONTY2R 479

namespace my_server {

class cudaOpticalFlow
{
public:
    cudaOpticalFlow();
    Mat imageproccess(Mat& frame);
    virtual ~cudaOpticalFlow();
    double getSteerVal();
    void setGain(double p_gain);
    PID pid;

private:
    Mat frame0;
    bool useGray, showLine;
    int winSize, maxLevel, iters, points;
    double minDist, quality;
    double leftError = 0, rightError = 0;
    double flowGain = 0.1, steerVal = 0;
    Mat frame0Gray, frame1Gray, tempFrame;

    GpuMat d_frame0Gray, d_prevPts, d_frame0, d_frame1, d_frame1Gray, d_nextPts, d_status;

    Point sumSkyCount;
    Point sumLeftGroundCount;
    Point sumRightGroundCount;
    Point skyRotation, leftGroundTranslation, rightGroundTranslation;
    Point topMiddle;
    Point center;
    Point bottomMidle;
    Point rightCenter;
    Point leftCenter;
    Point randomPoint1;
    Point randomPoint2;

    PyrLKOpticalFlow d_pyrLK;
    GoodFeaturesToTrackDetector_GPU detector;

    Point f1L, f1R, f2L, f2R;

    int collisionCount = 0, leftGroundCount = 0, rightGroundCount = 0, skyCount = 0, countFlow;
    double sumCollionTime = 0, sumMagnitude = 0, collisionTime = 0;

    void download(const GpuMat& d_mat, vector<Point2f>& vec);
    void download(const GpuMat& d_mat, vector<uchar>& vec);
    void drawArrows(Mat& frame, vector<Point2f>& prevPts, vector<Point2f>& nextPts, vector<uchar>& status, Scalar line_color = Scalar(0, 0, 255));
};

}

#endif
