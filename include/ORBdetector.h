#ifndef ORBDETECTOR_H
#define ORBDETECTOR_H

#include <iostream>
#include <vector>
#include <opencv/cv.h>
using namespace std;
using namespace cv;
namespace Mono_vo
{
    class ORBdetector
    {
    private:
        /* data */
    public:
        void find_feature_matches(
            const Mat& img_1, const Mat& img_2,
            std::vector<KeyPoint>& keypoints_1,
            std::vector<KeyPoint>& keypoints_2,
            std::vector<DMatch>& matches
        );

    public:
        ORBdetector(/* args */);
        ~ORBdetector();
    };
    
    ORBdetector::ORBdetector(/* args */)
    {
    }
    
    ORBdetector::~ORBdetector()
    {
    }
    
}
#endif