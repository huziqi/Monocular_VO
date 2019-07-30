#include "Camera.h"
#include "Config.h"

namespace Mono_vo
{
    
Camera::Camera()
{
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");

}

Vector3d Camera::world2camera ( const Vector3d& p_w, const Eigen::Matrix4d& T_c_w )
{
    Eigen::Vector3d t;
    Eigen::Matrix3d r;
    t<<T_c_w(0,3),T_c_w(1,3),T_c_w(2,3);
    r<<T_c_w(0,0),T_c_w(0,1),T_c_w(0,2),
       T_c_w(1,0),T_c_w(1,1),T_c_w(1,2),
       T_c_w(2,0),T_c_w(2,1),T_c_w(2,2);
    return r*p_w+t;
}

Vector3d Camera::camera2world ( const Vector3d& p_c, const Eigen::Matrix4d& T_c_w )
{
    Eigen::Vector3d t;
    Eigen::Matrix3d r;
    t<<T_c_w(0,3),T_c_w(1,3),T_c_w(2,3);
    r<<T_c_w(0,0),T_c_w(0,1),T_c_w(0,2),
       T_c_w(1,0),T_c_w(1,1),T_c_w(1,2),
       T_c_w(2,0),T_c_w(2,1),T_c_w(2,2);
    return r.inverse() *(p_c-t);
}

Vector2d Camera::camera2pixel ( const Vector3d& p_c )
{
    return Vector2d (
               fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
               fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
           );
}

Point2f Camera::pixel2camera ( const Point2d& p, const Mat& K )
{
    return Point2f
        (
            ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0), 
            ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1) 
        );
}


Vector2d Camera::world2pixel ( const Vector3d& p_w, const Eigen::Matrix4d& T_c_w )
{
    return camera2pixel ( world2camera(p_w, T_c_w) );
}

// Vector3d Camera::pixel2world ( const Vector2d& p_p, const Eigen::Matrix3d& T_c_w, double depth )
// {
//     return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
// }


}
