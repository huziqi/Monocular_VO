#ifndef FRAME_H
#define FRAME_H

#include "Common_include.h"
#include "Camera.h"
#include "MapPoint.h"

namespace Mono_vo 
{
    
// forward declare 
class MapPoint;
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long                  id_;         // id of this frame
    double                         time_stamp_; // when it is recorded
    Eigen::Matrix4d                T_c_w_;      // transform from world to camera
    Camera::Ptr                    camera_;     // Pinhole RGBD Camera model 
    Mat                            color_, depth_; // color and depth image 
    // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
    // std::vector<MapPoint*>         map_points_; // associated map points
    bool                           is_key_frame_;  // whether a key-frame
    vector<MapPoint::Ptr>              mappoint_inframe;
    vector<unsigned long>          mappoint_id;
    
public: // data members 
    Frame();
    Frame( long id, double time_stamp=0, Eigen::Matrix3d T_c_w=Eigen::Matrix3d::Identity(), Camera::Ptr camera=nullptr, Mat color=Mat());
    ~Frame();
    
    static Frame::Ptr createFrame(); 
    
    // find the depth in depth map
    double findDepth( const cv::KeyPoint& kp );
    
    // Get Camera Center
    Vector3d getCamCenter() const;
    
    void setPose( const Eigen::Matrix4d& T_c_w );
    
    // check if a point is in this frame 
    bool isInFrame( const Vector3d& pt_world );
};

}

#endif // FRAME_H