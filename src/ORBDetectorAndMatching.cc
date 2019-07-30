#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv_modules.hpp>
#include <ctime>

#include "ORBdetector.h"

using namespace std;
using namespace cv;

namespace Mono_vo
{
    void ORBdetector::find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
    {
        //-- 初始化
        Mat descriptors_1, descriptors_2;
        // used in OpenCV3 
        Ptr<FeatureDetector> detector = ORB::create();
        Ptr<DescriptorExtractor> descriptor = ORB::create();
        // use this if you are in OpenCV2 
        // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
        // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
        Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");
        //-- 第一步:检测 Oriented FAST 角点位置
        detector->detect ( img_1,keypoints_1 );
        detector->detect ( img_2,keypoints_2 );

        //-- 第二步:根据角点位置计算 BRIEF 描述子
        descriptor->compute ( img_1, keypoints_1, descriptors_1 );
        descriptor->compute ( img_2, keypoints_2, descriptors_2 );

        //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
        vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
        matcher->match ( descriptors_1, descriptors_2, match );

        //-- 第四步:匹配点对筛选
        double min_dist=10000, max_dist=0;

        //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
        for ( int i = 0; i < descriptors_1.rows; i++ )
        {
            double dist = match[i].distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist;
        }

        //printf ( "-- Max dist : %f \n", max_dist );
        //printf ( "-- Min dist : %f \n", min_dist );

        //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
        for ( int i = 0; i < descriptors_1.rows; i++ )
        {
            if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
            {
                matches.push_back ( match[i] );
            }
        }
    }

}