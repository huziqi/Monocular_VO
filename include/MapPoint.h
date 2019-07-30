#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "Common_include.h"
#include "Frame.h"
#include <map>

namespace Mono_vo
{   

class Frame;
class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long      id_;        // ID
    static unsigned long factory_id_;    // factory id
    bool        good_;      // wheter a good point 
    Point3d     pos_;       // Position in world
    Vector3d    norm_;      // Normal of viewing direction 
    Mat         descriptor_; // Descriptor for matching 
    

    vector<shared_ptr<Frame>>              observed_frames_;   // key-frames that can observe this point 
    vector<Point2d>                coordinate_inframe;
    unordered_map<unsigned long, Point2d> pt_2d_inframe;
    
    int         matched_times_;     // being an inliner in pose estimation
    int         visible_times_;     // being visible in current frame 
    
    MapPoint();
    MapPoint( 
        unsigned long id, 
        const Point3d& position, 
        const Vector3d& norm 
//        Frame* frame=nullptr, 
//        const Mat& descriptor=Mat() 
    );
    
    inline cv::Point3f getPositionCV() const {
        return cv::Point3f( pos_.x, pos_.y, pos_.z );
    }
    
    static MapPoint::Ptr createMapPoint();
    static MapPoint::Ptr createMapPoint( 
        const Point3d& pos_world, 
        const Vector3d& norm_,
        const Mat& descriptor,
        Frame* frame );
};
}

#endif // MAPPOINT_H
