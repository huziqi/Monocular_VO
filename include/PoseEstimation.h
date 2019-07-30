#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H
#include <iostream>
#include <opencv/cv.h>
#include <vector>

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

    public:
        PoseEstimation(/* args */){}
        ~PoseEstimation(){}
    };
    
    
}
#endif