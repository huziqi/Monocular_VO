#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H
#include <iostream>
#include <opencv/cv.h>
#include <vector>
#include "Config.h"

using namespace std;
using namespace cv;
namespace Mono_vo
{
    class PoseEstimation
    {
    private:
        /* data */
    public:
        void pose_estimation_2d2d (
        const std::vector<KeyPoint>& keypoints_1,
        const std::vector<KeyPoint>& keypoints_2,
        const std::vector< DMatch >& matches,
        Mat& R, Mat& t );

        Point2f pixel2cam ( const Point2d& p, const Mat& K );

        void triangulation ( 
        const Mat& T1,
        const vector< KeyPoint >& keypoint_1, 
        const vector< KeyPoint >& keypoint_2, 
        const std::vector< DMatch >& matches,
        const Mat& R, const Mat& t, 
        vector< Point3d >& points );

        float fx_, fy_, cx_, cy_;

    public:
        PoseEstimation(/* args */){
            fx_ = Config::get<float>("camera.fx");
            fy_ = Config::get<float>("camera.fy");
            cx_ = Config::get<float>("camera.cx");
            cy_ = Config::get<float>("camera.cy");
        }
        ~PoseEstimation(){}
    };
    
    
}
#endif