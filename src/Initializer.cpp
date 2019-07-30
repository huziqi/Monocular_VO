#include "Common_include.h"
#include "Initializer.h"
#include "PoseEstimation.h"


namespace Mono_vo
{
    double Initializer::MeanBias(
        const std::vector<KeyPoint> keypoints_1,
        const std::vector<KeyPoint> keypoints_2,
        const std::vector< DMatch > goodmatches)
    {
        double bias=0;
        double mean1_x, mean1_y, sum1_x, sum1_y;
        double mean2_x, mean2_y, sum2_x, sum2_y;
        for(int i=0;i<goodmatches.size();i++)
        {
            sum1_x+=keypoints_1[goodmatches[i].queryIdx].pt.x;
            sum1_y+=keypoints_1[goodmatches[i].queryIdx].pt.y;
            sum2_x+=keypoints_2[goodmatches[i].trainIdx].pt.x;
            sum2_y+=keypoints_2[goodmatches[i].trainIdx].pt.y;
        }
        mean1_x=sum1_x/goodmatches.size();
        mean1_y=sum1_y/goodmatches.size();
        mean2_x=sum2_x/goodmatches.size();
        mean2_y=sum2_y/goodmatches.size();

        return ((mean1_x-mean2_x)*(mean1_x-mean2_x)+(mean1_y-mean2_y)*(mean1_y-mean2_y))/10;
    }

    bool Initializer::Initialize_over(
        const std::vector<KeyPoint> keypoints_1,
        const std::vector<KeyPoint> keypoints_2,
        const std::vector< DMatch > goodmatches)
    {
        double Sh,Sf;
        double Rh;
        Sf=ComputeFundamental(keypoints_1,keypoints_2,goodmatches);
        Sh=ComputeHomograph(keypoints_1,keypoints_2,goodmatches);
        Rh=Sh/(Sh+Sf);
        if(Rh>0.45) return false;
        else
        {
            return true;
        }   
    }

    double Initializer::ComputeFundamental(
        const std::vector<KeyPoint> keypoints_1,
        const std::vector<KeyPoint> keypoints_2,
        const std::vector< DMatch > goodmatches
    )
    {
        vector<Point2d> ref, cur;
        for(int i=0;i<goodmatches.size();i++)
        {
            ref.push_back( keypoints_1[ goodmatches[i].queryIdx ].pt );  
            cur.push_back( keypoints_2[ goodmatches[i].trainIdx ].pt );      
        }
        Mat F = findFundamentalMat ( ref, cur, CV_FM_8POINT );
        float sigma=1.0;

        const double f11 = F.at<double>(0,0);
        const double f12 = F.at<double>(0,1);
        const double f13 = F.at<double>(0,2);
        const double f21 = F.at<double>(1,0);
        const double f22 = F.at<double>(1,1);
        const double f23 = F.at<double>(1,2);
        const double f31 = F.at<double>(2,0);
        const double f32 = F.at<double>(2,1);
        const double f33 = F.at<double>(2,2);

        double score = 0;

        const double th = 3.841;
        const double thScore = 5.991;

        const double invSigmaSquare = 1.0/(sigma*sigma);

        for(int i=0; i<goodmatches.size(); i++)
        {
            bool bIn = true;

            const cv::KeyPoint &kp1 = keypoints_1[goodmatches[i].queryIdx];
            const cv::KeyPoint &kp2 = keypoints_2[goodmatches[i].trainIdx];

            const double u1 = kp1.pt.x;
            const double v1 = kp1.pt.y;
            const double u2 = kp2.pt.x;
            const double v2 = kp2.pt.y;

            // Reprojection error in second image
            // l2=Fx1=(a2,b2,c2)

            const double a2 = f11*u1+f12*v1+f13;
            const double b2 = f21*u1+f22*v1+f23;
            const double c2 = f31*u1+f32*v1+f33;

            const double num2 = a2*u2+b2*v2+c2;

            const double squareDist1 = num2*num2/(a2*a2+b2*b2);

            const double chiSquare1 = squareDist1*invSigmaSquare;

            if(chiSquare1<th)
                score += thScore - chiSquare1;

            // Reprojection error in second image
            // l1 =x2tF=(a1,b1,c1)

            const double a1 = f11*u2+f21*v2+f31;
            const double b1 = f12*u2+f22*v2+f32;
            const double c1 = f13*u2+f23*v2+f33;

            const double num1 = a1*u1+b1*v1+c1;

            const double squareDist2 = num1*num1/(a1*a1+b1*b1);

            const double chiSquare2 = squareDist2*invSigmaSquare;

            if(chiSquare2<th)
                score += thScore - chiSquare2;
        }
        return score;
    }

    double Initializer::ComputeHomograph(
        const std::vector<KeyPoint> keypoints_1,
        const std::vector<KeyPoint> keypoints_2,
        const std::vector< DMatch > goodmatches
    )
    {
        vector<Point2d> ref, cur;
        float sigma=1.0;
        for(int i=0;i<goodmatches.size();i++)
        {
            ref.push_back( keypoints_1[ goodmatches[i].queryIdx ].pt );  
            cur.push_back( keypoints_2[ goodmatches[i].trainIdx ].pt );      
        }
        Mat H = findHomography( ref, cur, CV_RANSAC );  
        Mat H_=H.inv();
        const double h11 = H.at<double>(0,0);
        const double h12 = H.at<double>(0,1);
        const double h13 = H.at<double>(0,2);
        const double h21 = H.at<double>(1,0);
        const double h22 = H.at<double>(1,1);
        const double h23 = H.at<double>(1,2);
        const double h31 = H.at<double>(2,0);
        const double h32 = H.at<double>(2,1);
        const double h33 = H.at<double>(2,2);

        const double h11inv = H_.at<double>(0,0);
        const double h12inv = H_.at<double>(0,1);
        const double h13inv = H_.at<double>(0,2);
        const double h21inv = H_.at<double>(1,0);
        const double h22inv = H_.at<double>(1,1);
        const double h23inv = H_.at<double>(1,2);
        const double h31inv = H_.at<double>(2,0);
        const double h32inv = H_.at<double>(2,1);
        const double h33inv = H_.at<double>(2,2);


        double score = 0;

        const double th = 5.991;

        const double invSigmaSquare = 1.0/(sigma*sigma);

        for(int i=0; i<goodmatches.size(); i++)
        {
            bool bIn = true;

            const cv::KeyPoint &kp1 = keypoints_1[goodmatches[i].queryIdx];
            const cv::KeyPoint &kp2 = keypoints_2[goodmatches[i].trainIdx];

            const double u1 = kp1.pt.x;
            const double v1 = kp1.pt.y;
            const double u2 = kp2.pt.x;
            const double v2 = kp2.pt.y;

            // Reprojection error in first image
            // x2in1 = H_*x2

            const double w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
            const double u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
            const double v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

            const double squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

            const double chiSquare1 = squareDist1*invSigmaSquare;

            if(chiSquare1<th)
                score += th - chiSquare1;

            // Reprojection error in second image
            // x1in2 = H*x1

            const double w1in2inv = 1.0/(h31*u1+h32*v1+h33);
            const double u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
            const double v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

            const double squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

            const double chiSquare2 = squareDist2*invSigmaSquare;

            if(chiSquare2<th)
                score += th - chiSquare2;
        }
        return score;
    }

    void Initializer::ConstructPoints(const Mono_vo::Map::Ptr &map, 
        const std::vector<KeyPoint> keypoints_1,
        const std::vector<KeyPoint> keypoints_2,
        const std::vector< DMatch > goodmatches,
        const Mono_vo::Frame::Ptr& refframe,
        const Mono_vo::Frame::Ptr& curframe)
        {
            vector<Point3d> pts_3d;
            vector<Point2d> pts_2d;
            Mat R,t;
            Mat T1 = (Mat_<float> (3,4) <<
            1,0,0,0,
            0,1,0,0,
            0,0,1,0);

            Mono_vo::PoseEstimation pose_estimation1;
            //triangulation
            pose_estimation1.pose_estimation_2d2d(keypoints_1,keypoints_2,goodmatches,R,t);
            pose_estimation1.triangulation(T1,keypoints_1,keypoints_2,goodmatches,R,t,pts_3d);
            //draw_Points_3D.insert(Points_3D.end(),pts_3d.begin(),pts_3d.end());

            for(int i=0;i<pts_3d.size();i++)
            {
                Mono_vo::MapPoint::Ptr mappoint=Mono_vo::MapPoint::createMapPoint();
                mappoint->pos_=pts_3d[i];
                mappoint->observed_frames_.push_back(refframe);
                mappoint->coordinate_inframe.push_back(keypoints_1[goodmatches[i].queryIdx].pt);
                mappoint->pt_2d_inframe[refframe->id_]=keypoints_1[goodmatches[i].queryIdx].pt;
                refframe->mappoint_id.push_back(mappoint->factory_id_);
                refframe->mappoint_inframe.push_back(mappoint);
                map->insertMapPoint(mappoint);
            }

            for(int i=0;i<pts_3d.size();i++)
            {
                map->map_points_[i++]->observed_frames_.push_back(curframe);
                map->map_points_[i++]->coordinate_inframe.push_back(keypoints_2[goodmatches[i].trainIdx].pt);
                map->map_points_[i++]->pt_2d_inframe[curframe->id_]=keypoints_2[goodmatches[i].trainIdx].pt;
                curframe->mappoint_id.push_back(map->map_points_[i++]->factory_id_);
                curframe->mappoint_inframe.push_back(map->map_points_[i++]);
            }

            Eigen::Matrix4d Twc;       
             Twc << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
                    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(0,1),
                    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(0,2),
                    0                , 0                , 0                , 1                ; 
            curframe->T_c_w_=Twc;
            map->insertKeyFrame(refframe);
            map->insertKeyFrame(curframe);
        }
}