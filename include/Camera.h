#ifndef CAMERA_H
#define CAMERA_H

#include "Common_include.h"

namespace Mono_vo
{

// Monocular camera model
class Camera
{
public:
    typedef std::shared_ptr<Camera> Ptr;
    float   fx_, fy_, cx_, cy_;

    Camera();
    Camera ( float fx, float fy, float cx, float cy) :
        fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy )
    {}

    // coordinate transform: world, camera, pixel
    Vector3d world2camera( const Vector3d& p_w, const Eigen::Matrix4d& T_c_w );
    Vector3d camera2world( const Vector3d& p_c, const Eigen::Matrix4d& T_c_w );
    Vector2d camera2pixel( const Vector3d& p_c );
    Point2f  pixel2camera ( const Point2d& p, const Mat& K ); 
    Vector3d pixel2world ( const Vector2d& p_p, const Eigen::Matrix4d& T_c_w, double depth=1 );
    Vector2d world2pixel ( const Vector3d& p_w, const Eigen::Matrix4d& T_c_w );

};

}
#endif // CAMERA_H
