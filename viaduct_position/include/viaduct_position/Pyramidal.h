#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>  
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <typeinfo>
#include <signal.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <tf/transform_broadcaster.h>
#include "mission_manager/mission_manager.h"

#define foreach BOOST_FOREACH

using namespace cv;
using namespace std;

void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)
{ 
    vector<float> err;					
    TermCriteria termcrit=TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 30, 0.01);

    calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, Size(15,15), 2, termcrit, 0, 0.001);

    int indexCorrection = 0;
    for(int i=0; i<status.size(); i++)
    {  
        Point2f pt = points2.at(i - indexCorrection);
        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	
        {
            if((pt.x<0)||(pt.y<0)) status.at(i) = 0;
            points1.erase(points1.begin() + (i - indexCorrection));
            points2.erase(points2.begin() + (i - indexCorrection));
            indexCorrection++;
        }
    }
}

void drawFeatures(Mat frame, vector<Point2f>& points)
{
    for( size_t i = 0; i < points.size(); i++ ) 
    {
        circle(frame, points[i], 0.6, Scalar(0, 0, 255), 10);
    }
}

void FASTfeatureDetection(Mat img_1, vector<Point2f>& points1)	
{   //uses FAST as of now, modify parameters as necessary
  vector<KeyPoint> keypoints_1;
  int fast_threshold = 50;
  bool nonmaxSuppression = true;
  FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
  KeyPoint::convert(keypoints_1, points1, vector<int>());
}

void boxFeature(Mat img, vector<Point2f>& points2, int ncols, int nrows)
{   
    vector<Point2f> points1;
    float width = img.cols, height = img.rows;
    Mat box(height/nrows,width/ncols, CV_8UC1);

    for(int ni = 0; ni < ncols; ni++)
    {
        for(int nj = 0; nj < nrows; nj++)
        {
            for(int i = 0; i < width/ncols; i++)
            {    
                for(int j = 0; j < height/nrows; j++)
                {
                    box.at<uint8_t>(j, i) = img.at<uint8_t>(j+nj*height/nrows, i+ni*width/ncols); 
                }
            }
            FASTfeatureDetection(box, points1);
            
            for(int k = 0; k < points1.size(); k++)
            {
                points1[k].x = points1[k].x + ni*width/ncols;
                points1[k].y = points1[k].y + nj*height/nrows;
            }

            points2.insert(points2.end(), points1.begin(), points1.end());

            points1.clear();
        }
    }
}

tuple<int, int> minFeatureBox(Mat img, vector<Point2f>& points, int ncols, int nrows, int threshold)
{
    std::tuple<int, int> myTuple; // empty tuple
    int minCont = 10000;
    float width = img.cols, height = img.rows;
    for(int ni = 0; ni < ncols; ni++)
    {
        for(int nj = 0; nj < nrows; nj++)
        {
            int cont = 0;
            for(int k = 0; k < points.size(); k++)
            {
                //printf("Punto[%d]:", k); 
                //cout << points[k] << endl; 
                if ((points[k].y > (nj*height/nrows)) && (points[k].y < (nj+1)*height/nrows) && (points[k].x > (ni*width/ncols)) && (points[k].x < (ni+1)*width/ncols))
                {
                    cont++;
                }
            }

            if (cont < minCont){
                // cout << "Puntos en la celda:" << cont << endl; 
                myTuple = make_tuple(ni, nj);
                minCont = cont;
            } 
        }
    }

    if (minCont < threshold) // si el cont de box minimo es inferior al limite, devuelve box
    {
        return myTuple;
    }
    else
    {
        return make_tuple(-1, -1);
    }
}