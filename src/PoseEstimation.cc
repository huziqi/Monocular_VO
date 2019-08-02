#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "PoseEstimation.h"

// #include "extra.h" // use this if in OpenCV2 
using namespace std;
using namespace cv;

namespace Mono_vo
{

    Point2f PoseEstimation::pixel2cam ( const Point2d& p, const Mat& K )
    {
        return Point2f
        (
            ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0), 
            ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1) 
        );
    }

    void PoseEstimation::triangulation ( const Mat& T1,
        const vector< KeyPoint >& keypoint_1, 
        const vector< KeyPoint >& keypoint_2, 
        const std::vector< DMatch >& matches,
        const Mat& R, const Mat& t, 
        vector< Point3d >& points )
    {
        // Mat T1 = (Mat_<float> (3,4) <<
        //     1,0,0,0,
        //     0,1,0,0,
        //     0,0,1,0);
        Mat T2 = (Mat_<float> (3,4) <<
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
        );
        
        Mat K = ( Mat_<double> ( 3,3 ) << fx_, 0, fy_, 0, cx_, cy_, 0, 0, 1 );
        vector<Point2f> pts_1, pts_2;
        for ( DMatch m:matches )
        {
            // 将像素坐标转换至相机坐标
            pts_1.push_back ( pixel2cam( keypoint_1[m.queryIdx].pt, K) );
            pts_2.push_back ( pixel2cam( keypoint_2[m.trainIdx].pt, K) );
        }
        
        Mat pts_4d;
        cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );
        
        // 转换成非齐次坐标
        for ( int i=0; i<pts_4d.cols; i++ )
        {
            Mat x = pts_4d.col(i);
            x /= x.at<float>(3,0); // 归一化
            Point3d p (
                x.at<float>(0,0), 
                x.at<float>(1,0), 
                x.at<float>(2,0) 
            );
            points.push_back( p );
        }
    }

    void PoseEstimation::pose_estimation_2d2d (
    const std::vector<KeyPoint>& keypoints_1,
    const std::vector<KeyPoint>& keypoints_2,
    const std::vector< DMatch >& matches,
    Mat& R, Mat& t )
    {
        // 相机内参,TUM Freiburg2
        Mat K = ( Mat_<double> ( 3,3 ) << fx_, 0, fy_, 0, cx_, cy_, 0, 0, 1 );

        //-- 把匹配点转换为vector<Point2f>的形式
        vector<Point2f> points1;
        vector<Point2f> points2;

        for ( int i = 0; i < ( int ) matches.size(); i++ )
        {
            points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
            points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
        }

        //-- 计算基础矩阵
        Mat fundamental_matrix;
        fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
        //cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

        //-- 计算本质矩阵
        Point2d principal_point ( cx_, cy_ );				//相机主点, TUM dataset标定值
        int focal_length = fx_;						//相机焦距, TUM dataset标定值
        Mat essential_matrix;
        essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
        //cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

        //-- 计算单应矩阵
        Mat homography_matrix;
        homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
        //cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

        //-- 从本质矩阵中恢复旋转和平移信息.
        recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
        //cout<<"R is "<<endl<<R<<endl;
        //cout<<"t is "<<endl<<t<<endl;
    }

    
}