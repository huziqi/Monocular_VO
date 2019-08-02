/*Author:Huziqi 
  Date:2019/7/25 */


#include <sophus/se3.h>
#include <sophus/so3.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <opencv2/viz.hpp>

#include "Common_include.h"
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
using Sophus::SO3;
using Sophus::SE3;


void LoadImages(const string &strImagePath, const string &strPathTimes,
                    vector<string> &vstrImages, vector<double> &vTimeStamps);
void DrawPoints(vector< Point3d >& points);
void DrawFrames(vector<Eigen::Matrix4d>& Twc);
void Goodmatches(vector<DMatch> &matches, vector<DMatch> &good_matches);
void bundleAdjustment (const vector< Point3d > points_3d,const vector< Point2d > points_2d,const Mat& K,Mat& R, Mat& t,Eigen::Matrix4d& Twc, Mat &inliers );


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
    int num_inliers=0;
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
    Map->insertKeyFrame(pFrame);
    Point3d point_begin(0.0, 0.0, 0.0);
    Point3d point_end;
    vector<cv::viz::WLine> lines; 
    //visualization of viz
    cv::viz::Viz3d vis ( "Visual Odometry" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose ( cam_pose );

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );

    for(int ni=1; ni<vstrImageFilenames.size(); ni++)
    //for(int ni=1; ni<150;ni++)
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
        
        if(!init->Initialize_over(keypoints_1,keypoints_2,good_matches)&&isInitializing)
        {
            continue;
        }
        

        if(isInitializing)
        {
            cout<<"start to construct first group of 3D points!!"<<endl;
            init->ConstructPoints(Map,keypoints_1,keypoints_2,good_matches,refframe,curframe);
            refframe=pFrame;
            isInitializing=false;
            cout<<"finish initializing!!!!!!!!!!!!"<<endl;
            cout<<"image num: "<<vstrImageFilenames[ni]<<endl;
            continue;
        }
        
        
    
        
        pts_2d.clear();
        pts_3d.clear();

        for(int i=0;i<good_matches.size();i++)
        {
            for(auto point:refframe->mappoint_inframe)
            {
                if(keypoints_1[good_matches[i].queryIdx].pt.x==point->pt_2d_inframe[refframe->id_].x&&
                    keypoints_1[good_matches[i].queryIdx].pt.y==point->pt_2d_inframe[refframe->id_].y)
                    {
                        pts_2d.push_back(keypoints_2[good_matches[i].trainIdx].pt);
                        pts_3d.push_back(point->pos_);
                    }
            }
        }
        cout<<"pnp pts_2d size: "<<pts_2d.size()<<endl;
        cout<<"pnp pts_3d size: "<<pts_3d.size()<<endl;
        cout<<endl;
    
        //pnp
        Mat inliers;
        cv::solvePnPRansac(pts_3d, pts_2d, K, Mat(), r, t, false, 100, 4.0, 0.99, inliers);

        //solvePnP ( pts_3d, pts_2d, K, Mat(), r, t, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
        
        num_inliers=inliers.rows;
        cout<<"PNP inliers:"<<num_inliers<<endl;
        SE3 T_c_w_estimated_ = SE3 (
                            SO3 ( r.at<double> ( 0,0 ), r.at<double> ( 1,0 ), r.at<double> ( 2,0 ) ),
                            Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 0,1 ), t.at<double> ( 0,2 ) )
                        );
        eigen2cv(Eigen::Matrix3d(T_c_w_estimated_.rotation_matrix()),R);
        // cout<<"R!!!"<<endl;
        // cout<<T_c_w_estimated_.rotation_matrix()<<endl;
        // cout<<endl;
        
        //cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵
        //cout<<"R: "<<endl<<R<<endl;
        //use BA optimizer
        Eigen::Matrix4d Twc;
        //bundleAdjustment(pts_3d,pts_2d,K,R,t,Twc,inliers);
        Twc<<R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0),
             R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(0,1),
             R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(0,2),
             0,                0,                0,                1                ;

        curframe->T_c_w_=Twc;
        cout<<"curframe's Twc is :"<<endl;
        cout<<curframe->T_c_w_<<endl;
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
                if(pts_3d[i].z<=0) continue;
                if(point->pt_2d_inframe[refframe->id_].x!=keypoints_1[good_matches[i].queryIdx].pt.x&&
                point->pt_2d_inframe[refframe->id_].y!=keypoints_1[good_matches[i].queryIdx].pt.y)
                {
                    // Eigen::Vector3d pts_3d_;
                    // pts_3d_<<pts_3d[i].x,pts_3d[i].y,pts_3d[i].z;
                    // Eigen::Vector3d t;
                    // Eigen::Matrix3d r;
                    // t<<refframe->T_c_w_(0,3),refframe->T_c_w_(1,3),refframe->T_c_w_(2,3);
                    // r<<refframe->T_c_w_(0,0),refframe->T_c_w_(0,1),refframe->T_c_w_(0,2),
                    //    refframe->T_c_w_(1,0),refframe->T_c_w_(1,1),refframe->T_c_w_(1,2),
                    //    refframe->T_c_w_(2,0),refframe->T_c_w_(2,1),refframe->T_c_w_(2,2);
                    // Vector3d world_pts_3d = r.inverse() *(pts_3d_-t);
                    // Point3d point_3D=Point3d(world_pts_3d[0],world_pts_3d[1],world_pts_3d[2]);
                    //new_pts_3d.push_back(point_3D);
                    new_pts_3d.push_back(pts_3d[i]);
                    break;
                }
            }
        }
        for(int i=0;i<pts_3d.size();i++)
        {
            for(auto point:refframe->mappoint_inframe)
            {
                if(point->pt_2d_inframe[refframe->id_].x==keypoints_1[good_matches[i].queryIdx].pt.x&&
                point->pt_2d_inframe[refframe->id_].y==keypoints_1[good_matches[i].queryIdx].pt.y)
                {
                    curframe->mappoint_id.push_back(point->factory_id_);
                    curframe->mappoint_inframe.push_back(point);
                }
            }
        }

        cout<<"add new 3D points' size is:"<<new_pts_3d.size()<<endl;
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
        
        

        cv::Affine3d M (
                cv::Affine3d::Mat3 (
                    Twc( 0,0 ), Twc( 0,1 ), Twc( 0,2 ),
                    Twc( 1,0 ), Twc( 1,1 ), Twc( 1,2 ),
                    Twc( 2,0 ), Twc( 2,1 ), Twc( 2,2 )
                ),
                cv::Affine3d::Vec3 (
                    Twc( 0,3 ), Twc( 1,3 ), Twc( 2,3 )
                )
            );
        point_end = Point3d(Twc(0, 3),Twc(1, 3),Twc(2, 3));
        viz::WLine line(point_begin, point_end, cv::viz::Color::green());
        lines.push_back(line);
        int j=0;
        for (vector<cv::viz::WLine>::iterator iter = lines.begin(); iter != lines.end(); iter++)
         {
             string id = to_string(j);
             vis.showWidget(id, *iter);
             j++;
         }
        point_begin = point_end; // 更新 起始点
        //drawMatches ( refframe->color_, keypoints_1, curframe->color_, keypoints_2, good_matches, img_goodmatch );
        //imshow ( "优化后匹配点对", img_goodmatch );
        //drawKeypoints( curframe->color_, keypoints_1, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
        //imshow("ORB特征点",outimg);
        //waitKey(1);
        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1500, false );
        //waitKey(1);
        //system("sleep 2s");
        cout << endl;
        //////////////////////////////////
    }

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

// void DrawPoints(vector< Point3d >& points){
//     glPointSize(5.0f);
//     glBegin(GL_POINTS);
//     for(size_t i = 0;i<points.size();++i){
//         glColor3d(0,0,0);
//         // glColor3f(float(cloud->points[i].r)/255,float( cloud->points[i].g)/255,float(cloud->points[i].b)/255);
//         glVertex3d(points[i].x,points[i].y,points[i].z);
//     }
//     glEnd();
// }
// void DrawFrames(vector<Eigen::Matrix4d>& Twc){
//     const float w = 0.08; //0.01
//     const float h = w * 0.75f;
//     const float z = w * 0.6f;

//     for(int i=0;i<Twc.size();i++)
//     {
        
//         glPushMatrix();
//         //glMultMatrixd(Twc[i].matrix().data());
//         glMultMatrixd(Twc[i].data());
//         glColor3d(0, (1 - 0), 0);
//         glLineWidth(2.0f);
//         glBegin(GL_LINES);
//         glVertex3f(0, 0, 0);
//         glVertex3f(w, h, z);
//         glVertex3f(0, 0, 0);
//         glVertex3f(w, -h, z);
//         glVertex3f(0, 0, 0);
//         glVertex3f(-w, -h, z);
//         glVertex3f(0, 0, 0);
//         glVertex3f(-w, h, z);
//         glVertex3f(w, h, z);
//         glVertex3f(w, -h, z);
//         glVertex3f(-w, h, z);
//         glVertex3f(-w, -h, z);
//         glVertex3f(-w, h, z);
//         glVertex3f(w, h, z);
//         glVertex3f(-w, -h, z);
//         glVertex3f(w, -h, z);
//         glEnd();
//         glPopMatrix();
//     }
// }

void bundleAdjustment (
    const vector< Point3d > points_3d,
    const vector< Point2d > points_2d,
    const Mat& K,
    Mat& R, Mat& t , Eigen::Matrix4d& Twc, Mat& inliers)
{
   // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverCSparse<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr) );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
            R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
            R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
            R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                        ) );
    optimizer.addVertex ( pose );

    int index = 1;
    for ( const Point3f p:points_3d )   // landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId ( index++ );
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        optimizer.addVertex ( point );
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // edges
    index = 1;
    for (  int i=0; i<inliers.rows; i++  )
    {
        int index = inliers.at<int> ( i,0 );
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( points_2d[index].x, points_2d[index].y ) );
        //edge->point_ = Vector3d ( points_3d[index].x, points_3d[index].y, points_3d[index].z );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;
    Eigen::Isometry3d temp=pose->estimate();

    Twc << (temp.matrix())(0,0), (temp.matrix())(0,1), (temp.matrix())(0,2), (temp.matrix())(0,0),
           (temp.matrix())(1,0), (temp.matrix())(1,1), (temp.matrix())(1,2), (temp.matrix())(0,1),
           (temp.matrix())(2,0), (temp.matrix())(2,1), (temp.matrix())(2,2), (temp.matrix())(0,2),
           0        , 0        , 0        , 1        ; 

}
