/*Author:Huziqi 
  Date:2019/7/25 */

#include <iostream>
#include <ctime>
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <algorithm>
#include <eigen3/Eigen/Core>

#include "PoseEstimation.h"
#include "ORBdetector.h"
#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Initializer.h"
#include "Config.h"


using namespace std;
using namespace cv;
using namespace Mono_vo;


void LoadImages(const string &strImagePath, const string &strPathTimes,
                    vector<string> &vstrImages, vector<double> &vTimeStamps);
void DrawPoints(vector< Point3d >& points);
void DrawFrames(vector<Eigen::Matrix4d>& Twc);
void Goodmatches(vector<DMatch> &matches, vector<DMatch> &good_matches);


int main(int argc, char **argv)
{
    if(argc!=4)
    {
        cerr<<endl<<"Usage:./run_Mono_vo path_to_image_folder path_to_times_file config_file" << endl;
        return 1;
    }
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[1]), string(argv[2]), vstrImageFilenames, vTimestamps);
    int nImages=vstrImageFilenames.size();
    Mono_vo::Config::setParameterFile ( argv[3] );

    if(nImages<=0)
    {
        cerr<<"ERROR: Failed to load images"<<endl;
        return 1;
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Vector for 3D_Points and trajectory of camera
    vector<Point3d> draw_Points_3D;
    vector<Eigen::Matrix4d> draw_Twc_trajectory;
    bool isInitializing=true;
    Mat K = ( Mat_<double> ( 3,3 ) << 458.654, 0, 367.215, 0, 457.296, 248.375, 0, 0, 1 );
    Mono_vo::Camera::Ptr camera (new Mono_vo::Camera());
    Mono_vo::Frame::Ptr refframe= Mono_vo::Frame::createFrame();
    Mono_vo::Frame::Ptr curframe= Mono_vo::Frame::createFrame();
    Mono_vo::Frame::Ptr pFrame= Mono_vo::Frame::createFrame();
    Mono_vo::Map::Ptr Map = shared_ptr<Mono_vo::Map>(new Mono_vo::Map());
    //pFrame->camera_=camera;
    pFrame->color_=cv::imread(vstrImageFilenames[0],CV_LOAD_IMAGE_UNCHANGED);
    pFrame->time_stamp_=vTimestamps[0];
    refframe=pFrame;
    cout<<"timestamp[0]: "<<pFrame->time_stamp_<<endl;

    cv::Mat img, outimg;
    vector<Mono_vo::Frame::Ptr> KeyFrames;
    KeyFrames.push_back(pFrame);
    for(int ni=1; ni<100; ni++)
    {
        // Read image from file
        cout<<"****** Loop "<<ni<<" ***************"<<endl;
        img = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        Mono_vo::Frame::Ptr pFrame= Mono_vo::Frame::createFrame();
        //pFrame->camera_=camera;
        pFrame->color_=img;
        pFrame->time_stamp_=vTimestamps[ni];
        curframe=pFrame;
        
        //cout<<"timestamp["<<ni<<"]: "<<pFrame->time_stamp_<<endl;
        
        if(img.empty())
        {
            cerr << endl << "Failed to load image at: "
                <<  vstrImageFilenames[ni] << endl;
            return 1;
        }
        KeyFrames.push_back(pFrame);
        Mono_vo::Initializer::Ptr init;

        vector<KeyPoint> keypoints_1, keypoints_2;
        vector<DMatch> matches, good_matches;
        ORBdetector feature_detector;
        Mat r,R,t;
        Mat img_match;
        Mat img_goodmatch;
        vector<Point3d> pts_3d;
        vector<Point2d> pts_2d;

        PoseEstimation pose_estimation;
        feature_detector.find_feature_matches ( refframe->color_,curframe->color_ , keypoints_1, keypoints_2, matches );
        Goodmatches(matches, good_matches);
        
        if(!init->Initialize_over(keypoints_1,keypoints_2,good_matches))
        {
            continue;
        }
        
        cout<<"start at "<<ni<<" frame";

        if(isInitializing)
        {
            cout<<"start to construct first group of 3D points!!"<<endl;
            init->ConstructPoints(Map,keypoints_1,keypoints_2,good_matches,refframe,curframe);
            refframe=pFrame;
            isInitializing=false;
            cout<<"finish initializing!!!!!!!!!!!!"<<endl;
            continue;
        }
        
        
        pts_2d.clear();
        pts_3d.clear();

        for(int i=0;i<good_matches.size();i++)
        {
            for(auto point:refframe->mappoint_inframe)
            {
                if(keypoints_1[good_matches[i].queryIdx].pt.x==point->pt_2d_inframe[curframe->id_].x&&
                    keypoints_1[good_matches[i].queryIdx].pt.y==point->pt_2d_inframe[curframe->id_].y)
                    {
                        pts_2d.push_back(keypoints_2[good_matches[i].trainIdx].pt);
                        pts_3d.push_back(point->pos_);
                    }
            }

            
        }
    
        //pnp
        solvePnP ( pts_3d, pts_2d, K, Mat(), r, t, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
        cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵

        Eigen::Matrix4d Twc;       
        Twc << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
               R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(0,1),
               R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(0,2),
               0                , 0                , 0                , 1                ; 
        curframe->T_c_w_=Twc;
        Map->insertKeyFrame(curframe);

        //triangulation
        Mat T1 = (Mat_<float> (3,4) <<refframe->T_c_w_(0,0), refframe->T_c_w_(0,1), refframe->T_c_w_(0,2), refframe->T_c_w_(0,3),
                                      refframe->T_c_w_(1,0), refframe->T_c_w_(1,1), refframe->T_c_w_(1,2), refframe->T_c_w_(1,3),
                                      refframe->T_c_w_(2,0), refframe->T_c_w_(2,1), refframe->T_c_w_(2,2), refframe->T_c_w_(2,3)
        );
        vector<Point3d> new_pts_3d;
        pts_3d.clear();
        pose_estimation.triangulation(T1,keypoints_1,keypoints_2,good_matches,R,t,pts_3d);
        for(int i=0;i<pts_3d.size();i++)
        {
            for(auto point:refframe->mappoint_inframe)
            {
                if(point->pt_2d_inframe[refframe->id_].x!=keypoints_1[good_matches[i].queryIdx].pt.x&&
                point->pt_2d_inframe[refframe->id_].y!=keypoints_1[good_matches[i].queryIdx].pt.y)
                {
                    new_pts_3d.push_back(pts_3d[i]);
                    break;
                }
            }
        }

        for(int i=0;i<new_pts_3d.size();i++)
            {
                Mono_vo::MapPoint::Ptr mappoint=Mono_vo::MapPoint::createMapPoint();
                mappoint->pos_=new_pts_3d[i];
                mappoint->observed_frames_.push_back(refframe);
                mappoint->coordinate_inframe.push_back(keypoints_1[good_matches[i].queryIdx].pt);
                mappoint->pt_2d_inframe[refframe->id_]=keypoints_1[good_matches[i].queryIdx].pt;
                mappoint->observed_frames_.push_back(curframe);
                mappoint->coordinate_inframe.push_back(keypoints_2[good_matches[i].trainIdx].pt);
                mappoint->pt_2d_inframe[curframe->id_]=keypoints_2[good_matches[i].trainIdx].pt;
                refframe->mappoint_id.push_back(mappoint->factory_id_);
                refframe->mappoint_inframe.push_back(mappoint);
                curframe->mappoint_id.push_back(mappoint->factory_id_);
                curframe->mappoint_inframe.push_back(mappoint);
                Map->insertMapPoint(mappoint);
            }
        //////////////////


        refframe=curframe;
            

        cout<<ni<<"->"<<ni++<<"Camera_Twc is "<<endl<<Twc<<endl;
        
//        drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
//        imshow ( "优化后匹配点对", img_goodmatch );
//        drawKeypoints( img_1, keypoints_1, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//        imshow("ORB特征点",outimg);
//        waitKey(2);
    }
/*
    //visualization of Pangolin
    pangolin::CreateWindowAndBind("Mono_VO",1024,768);
    glEnable(GL_DEPTH_TEST);

    //define menu
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menu("menu.test",true,true);
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
        pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0, pangolin::AxisY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    pangolin::GlTexture imageTexture(640,480,GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);

    bool bFollow=true;
    pangolin::OpenGlMatrix T;
    T.SetIdentity();

    while( !pangolin::ShouldQuit() )
    {
        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(T);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(T);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }
        d_cam.Activate(s_cam);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        DrawPoints(Points_3D);
        DrawFrames(Twc_trajectory);
        pangolin::FinishFrame();

        usleep(500);
    }
 */
    //////////////////////////////////


    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath+"/"+ss.str()+".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);
        }
    }
                    
}    

void Goodmatches(vector<DMatch> &matches, vector<DMatch> &good_matches)
{
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < matches.size(); i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < matches.size(); i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }
}

void DrawPoints(vector< Point3d >& points){
    glPointSize(5.0f);
    glBegin(GL_POINTS);
    for(size_t i = 0;i<points.size();++i){
        glColor3d(0,0,0);
        // glColor3f(float(cloud->points[i].r)/255,float( cloud->points[i].g)/255,float(cloud->points[i].b)/255);
        glVertex3d(points[i].x,points[i].y,points[i].z);
    }
    glEnd();
}
void DrawFrames(vector<Eigen::Matrix4d>& Twc){
    const float w = 0.08; //0.01
    const float h = w * 0.75f;
    const float z = w * 0.6f;

    for(int i=0;i<Twc.size();i++)
    {
        
        glPushMatrix();
        //glMultMatrixd(Twc[i].matrix().data());
        glMultMatrixd(Twc[i].data());
        glColor3d(0, (1 - 0), 0);
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();
        glPopMatrix();
    }
}