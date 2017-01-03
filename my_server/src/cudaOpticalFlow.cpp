#include <iostream>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <ctime>

#include "../include/my_server/cudaOpticalFlow.hpp"
#include "../include/my_server/PID.hpp"

using namespace std;
using namespace cv;
using namespace cv::gpu;

namespace my_server {

cudaOpticalFlow::cudaOpticalFlow(){
    useGray = false ;
    showLine = false;
    winSize = 11;
    maxLevel = 11;
    iters = 50;
    points = 500;
    minDist = 0.0;
    quality = 0.01;

    topMiddle = Point(centerX, 0);
    center = Point(centerX, centerY);
    bottomMidle = Point(centerX, yMax);
    rightCenter = Point(0, centerY);
    leftCenter = Point(xMax, centerY);
    randomPoint1 = Point(0, centerY - 100);
    randomPoint2 = Point(xMax, centerY - 100);

    detector.maxCorners = points;
    detector.qualityLevel = quality;
    detector.minDistance = minDist;
    detector.useHarrisDetector = false;

    d_pyrLK.winSize.width = winSize;
    d_pyrLK.winSize.height = winSize;
    d_pyrLK.maxLevel = maxLevel;
    d_pyrLK.iters = iters;

    f1L = Point(FRONTX1L, FRONTY1L);
    f1R = Point(FRONTX1R, FRONTY1R);
    f2L = Point(FRONTX2L, FRONTY2L);
    f2R = Point(FRONTX2R, FRONTY2R);
}

double cudaOpticalFlow::getSteerVal()
{
    return steerVal;
}

void cudaOpticalFlow::download(const GpuMat &d_mat, vector<Point2f> &vec)
{
    vec.resize(d_mat.cols);
    Mat mat(1, d_mat.cols, CV_32FC2, (void*)&vec[0]);
    d_mat.download(mat);
}

void cudaOpticalFlow::download(const GpuMat &d_mat, vector<uchar> &vec)
{
    vec.resize(d_mat.cols);
    Mat mat(1, d_mat.cols, CV_8UC1, (void*)&vec[0]);
    d_mat.download(mat);
}

double calcMagnitude(Point p1, Point q1)
{
    return sqrt( (double)(p1.y - q1.y)*(p1.y - q1.y) + (double)(p1.x - q1.x)*(p1.x - q1.x) );
}

void cudaOpticalFlow::drawArrows(Mat &frame, vector<Point2f> &prevPts, vector<Point2f> &nextPts,  vector<uchar> &status, Scalar line_color)
{
    skyCount = 0;
    sumSkyCount.x = 0;
    sumSkyCount.y = 0;
    leftGroundCount = 0; rightGroundCount = 0;
    sumLeftGroundCount.x = 0; sumLeftGroundCount.y = 0;
    sumRightGroundCount.x = 0; sumRightGroundCount.y = 0;
    collisionCount = 0;
    sumCollionTime = 0;

    leftError = 0;
    rightError = 0;
    countFlow = 0;

    int k = 0, featureFound = 0;
    double meanFlow, sumMagnitude = 0;
    double flowLeft = 0, flowRight = 0;

    int sizePts = nextPts.size();

    if(showLine)
    {
        line(frame, f1L, f2L, Scalar(0,255,255));
        line(frame, f1R, f2R, Scalar(0,255,255));
    }

    //line(frame, center, bottomMidle, Scalar(0, 255, 255));
    //line(frame, rightCenter, leftCenter, Scalar(0, 255, 255));
    //line(frame, randomPoint1, randomPoint2, Scalar(0, 255, 255));

    /*
    for(size_t i = 0; i < sizePts; i++)
    {
        Point p = p_prevPts[i];
        Point q = p_nextPts[i];

        double hypotenuse = calcMagnitude(p, q);
        if(hypotenuse > 20.0 || hypotenuse < -20.0)
            continue;

        sumMagnitude += hypotenuse;

    }

    double meanHypo = sumMagnitude / sizePts;
    cout << meanHypo << endl;
    if(meanHypo == 0)
    {
        cout << "Err! " << sumMagnitude << "\t" << sizePts << endl;
        return;
    }*/

    for (size_t i = 0; i < sizePts; i++)
    {
        if (status[i])
        {
            int line_thickness = 1;

            //double angle = atan2((double) p.y - q.y, (double) p.x - q.x);

            Point p = prevPts[i];
            Point q = nextPts[i];

            double hypotenuse = calcMagnitude(p, q);

            if(hypotenuse > 50.0 || hypotenuse < 10.0)
                continue;

            nextPts[k].x = nextPts[i].x;
            nextPts[k].y = nextPts[i].y;

            prevPts[k].x = prevPts[i].x;
            prevPts[k].y = prevPts[i].y;

            status[k] = status[i];
            k++;

            /*
            // Here we lengthen the arrow by a factor of three.
            q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
            q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

            // Now we draw the main line of the arrow.
            line(frame, p, q, line_color, line_thickness);

            // Now draw the tips of the arrow. I do some scaling so that the
            // tips look proportional to the main line of the arrow.

            p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
            p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
            line(frame, p, q, line_color, line_thickness);

            p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
            p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
            line(frame, p, q, line_color, line_thickness);*/
        }
    }

    if(k == 0)
    {
        steerVal = 0;

        stringstream convert1, convert2, convert3;
        convert3 << steerVal;
        putText(frame, "Steer: "+ convert3.str(), Point(50, 340), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(0,255,255));
        convert1 << flowLeft;
        putText(frame, "Left: " + convert1.str(), Point(50, 370), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(0,255,255));
        convert2 << flowRight;
        putText(frame, "Right: "+ convert2.str(), Point(50, 400), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(0,255,255));

        pid.compute((steerVal));
        return;
    }

    featureFound = k;
    meanFlow = sumMagnitude / (double)k;
    //cout << "Mean Flow: " << meanFlow << endl;

    Point size = frame.size();

    for(size_t i = 0; i < featureFound ; i++)
    {
        Point p0 = prevPts[i];
        Point p1 = nextPts[i];

        //cout << i << "\t" << p0 << "\t" << p1 << endl;

        //sky flow count
        if ((p0.y < (size.y * 2 / 7)))
        {
            skyCount++;
            sumSkyCount.x += (p1.x - p0.x);
            sumSkyCount.y += (p1.y - p0.y);
        }

        //left ground flow count
        if ((p1.y > (size.y * 2 / 7)) && (p1.x < (size.x / 2)))
        {
            leftGroundCount++;
            sumLeftGroundCount.x += (p1.x - p0.x);
            sumLeftGroundCount.y += (p1.y - p0.y);
        }

        //right ground flow count
        if ((p1.y > (size.y * 2 / 7)) && (p1.x > (size.x / 2)))
        {
            rightGroundCount++;
            sumRightGroundCount.x += (p1.x - p0.x);
            sumRightGroundCount.y += (p1.y - p0.y);
        }

        //collision flow in the middle of the frame
        if ((p1.y > (size.y * 2 / 7)) && (p1.x > size.x * 2 / 7) &&
            (p1.y < (size.y * 5 / 7)) && (p1.x < size.x * 5 / 7))
        {
            collisionCount++;
            sumCollionTime += (p1.y - p0.y);
        }

        if (nextPts[i].x < centerX)
        {
            arrowedLine(frame, p0, p1, CV_RGB(0, 255, 255), 2);
        }
        else if (nextPts[i].x > centerX)
        {
            arrowedLine(frame, p0, p1, CV_RGB(0, 255, 0), 2);
        }
    }

    //sky rotation
    if (skyCount != 0)
    {
        skyRotation.x = sumSkyCount.x / skyCount;
        skyRotation.y = sumSkyCount.y / skyCount;
    }
    else
    {
        skyRotation.x = 0; skyRotation.y = 0;
    }
    Point p, q;

    p.x = size.x / 2; p.y = size.y * 1 / 7;
    q.x = (int)(p.x + skyRotation.x);
    q.y = (int)(p.y + skyRotation.y);

    arrowedLine(frame, p, q, Scalar(255, 255, 0), 4);

    //left ground rotation
    if(leftGroundCount != 0)
    {
        leftGroundTranslation.x = sumLeftGroundCount.x / leftGroundCount;
        leftGroundTranslation.y = sumLeftGroundCount.y / leftGroundCount;
    }else
    {
        leftGroundTranslation.y = 0;
        leftGroundTranslation.x = 0;
    }

    p.x = (int)(size.x / 4);
    p.y = (int)(size.y * 6 / 7);
    q.x = (int)(p.x + leftGroundTranslation.x - skyRotation.x);
    q.y = (int)(p.y + leftGroundTranslation.y - skyRotation.y);

    arrowedLine(frame, p, q, Scalar(255, 255, 0), 4);

    leftError += calcMagnitude(p, q);

    //right ground rotation
    if(rightGroundCount != 0)
    {
        rightGroundTranslation.x = sumRightGroundCount.x / rightGroundCount;
        rightGroundTranslation.y = sumRightGroundCount.y / rightGroundCount;
    }
    else
    {
        rightGroundTranslation.y = 0;
        rightGroundTranslation.x = 0;
    }

    p.x = (int)(size.x * 3 / 4);
    p.y = (int)(size.y * 6 / 7);
    q.x = (int)(p.x + rightGroundTranslation.x - skyRotation.x);
    q.y = (int)(p.y + rightGroundTranslation.y - skyRotation.y);

    arrowedLine(frame, p, q, Scalar(255, 255, 0), 4);

    if (collisionCount != 0)
        collisionTime = sumCollionTime / collisionCount;
    else collisionCount = 0;

    rightError += calcMagnitude(p, q);

    ////////////////////////////////////////////////////////////////////////////
    ///////calculate steering flow//////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    CvPoint2D32f steeringFromFlow;

    if (skyCount > 0 && leftGroundCount > 0 && rightGroundCount > 0)
    {
        CvPoint2D32f trLeft, trRight;

        trLeft.x = leftGroundTranslation.x;
        trLeft.y = leftGroundTranslation.y;

        trRight.x = rightGroundTranslation.x;
        trRight.y = rightGroundTranslation.y;

        flowLeft = -(trLeft.x - skyRotation.x);
        flowRight = trRight.x - skyRotation.x;

        steeringFromFlow.x = flowGain * ((flowLeft - flowRight) + skyRotation.x);
        steeringFromFlow.y = 0;

        steerVal = steeringFromFlow.x;
    }

    pid.compute(steerVal);

    stringstream convert1, convert2, convert3;
    convert3 << steerVal;
    putText(frame, "Steer: "+ convert3.str(), Point(50, 340), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(0,255,255));
    convert1 << flowLeft;
    putText(frame, "Left: " + convert1.str(), Point(50, 370), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(0,255,255));
    convert2 << flowRight;
    putText(frame, "Right: "+ convert2.str(), Point(50, 400), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(0,255,255));

    //cout << "L: " << flowLeft << "\t" << "R: " << flowRight << "\t";

}

template <typename T> inline T clamp (T x, T a, T b)
{
    return ((x) > (a) ? ((x) < (b) ? (x) : (b)) : (a));
}

template <typename T> inline T mapValue(T x, T a, T b, T c, T d)
{
    x = clamp(x, a, b);
    return c + (d - c) * (x - a) / (b - a);
}

void cudaOpticalFlow::setGain(double p_gain)
{
    flowGain = p_gain;
}

void callBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
     {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
}

Mat cudaOpticalFlow::imageproccess(Mat &frame1){

    if(frame0.empty())
    {
        frame0 = frame1.clone();
    }

    Mat frame0Gray;
    cvtColor(frame0, frame0Gray, COLOR_BGR2GRAY);
    Mat frame1Gray;
    cvtColor(frame1, frame1Gray, COLOR_BGR2GRAY);

    GpuMat d_frame0Gray(frame0Gray);
    GpuMat d_frame1Gray(frame1Gray);
    GpuMat d_prevPts;

    //gpu::bilateralFilter(d_frame0Gray, d_frame0Gray, 9, 0.1, 0.1);
    //gpu::bilateralFilter(d_frame1Gray, d_frame1Gray, 9, 0.1, 0.1);

    /*
    gpu::GaussianBlur(d_frame0Gray, d_frame0Gray, cv::Size(0, 0), 3);
    gpu::addWeighted(d_frame0Gray, 1.5, d_frame0Gray, -0.5, 0, d_frame0Gray);

    gpu::GaussianBlur(d_frame1Gray, d_frame1Gray, cv::Size(0, 0), 3);
    gpu::addWeighted(d_frame1Gray, 1.5, d_frame1Gray, -0.5, 0, d_frame1Gray);*/

    // goodFeaturesToTrack
    detector(d_frame0Gray, d_prevPts);
    detector(d_frame1Gray, d_nextPts);

    // Sparse

    GpuMat d_frame0(frame0);
    GpuMat d_frame1(frame1);
    GpuMat d_nextPts;
    GpuMat d_status;

    d_pyrLK.sparse(useGray ? d_frame0Gray : d_frame0, useGray ? d_frame1Gray : d_frame1, d_prevPts, d_nextPts, d_status);

    // Draw arrows

    vector<Point2f> prevPts(d_prevPts.cols);
    download(d_prevPts, prevPts);

    vector<Point2f> nextPts(d_nextPts.cols);
    download(d_nextPts, nextPts);

    vector<uchar> status(d_status.cols);
    download(d_status, status);

    tempFrame = frame1.clone();

    drawArrows(tempFrame, prevPts, nextPts, status, Scalar(0, 255, 255));

    /*
    imshow("click", frame0);
    setMouseCallback("click", callBackFunc, NULL);
    */

    frame0 = frame1.clone();

    return tempFrame;
}

cudaOpticalFlow::~cudaOpticalFlow(){
    d_pyrLK.releaseMemory();
    detector.releaseMemory();
}

}
