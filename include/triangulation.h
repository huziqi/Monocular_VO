#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <iostream>
#include <opencv/cv.h>
#include <vector>

using namespace std;
using namespace cv;

namespace Mono_vo
{
    class Triangulation
    {
    private:
        /* data */
    public:
        void triangulation (
        const vector<KeyPoint>& keypoint_1,
        const vector<KeyPoint>& keypoint_2,
        const std::vector< DMatch >& matches,
        const Mat& R, const Mat& t,
        vector<Point3d>& points
        );
    public:
        Triangulation(/* args */);
        ~Triangulation();
    };
    
    Triangulation::Triangulation(/* args */)
    {
    }
    
    Triangulation::~Triangulation()
    {
    }
    
}


#endif