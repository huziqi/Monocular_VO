#ifndef INITIALIZER_H
#define INITIALIZER_H

#include <opencv2/opencv.hpp>
#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"


namespace Mono_vo
{

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
class Initializer
{
    public:
    typedef std::shared_ptr<Initializer> Ptr;
    Initializer();

    double MeanBias(
        const std::vector<KeyPoint> keypoints_1,
        const std::vector<KeyPoint> keypoints_2,
        const std::vector< DMatch > goodmatches);

    double ComputeFundamental(
        const std::vector<KeyPoint> keypoints_1,
        const std::vector<KeyPoint> keypoints_2,
        const std::vector< DMatch > goodmatches
    );

    double ComputeHomograph(
        const std::vector<KeyPoint> keypoints_1,
        const std::vector<KeyPoint> keypoints_2,
        const std::vector< DMatch > goodmatches
    );

    bool Initialize_over(
        const std::vector<KeyPoint> keypoints_1,
        const std::vector<KeyPoint> keypoints_2,
        const std::vector< DMatch > goodmatches);

    void Initializer::ConstructPoints(const Mono_vo::Map::Ptr &map, 
        const std::vector<KeyPoint> keypoints_1,
        const std::vector<KeyPoint> keypoints_2,
        const std::vector< DMatch > goodmatches,
        const Mono_vo::Frame::Ptr& refframe,
        const Mono_vo::Frame::Ptr& curframe);

};

} //namespace ORB_SLAM

#endif // INITIALIZER_H
