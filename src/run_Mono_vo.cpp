/*Author:Huziqi 
  Date:2019/7/25 */

#include <pangolin/pangolin.h>
#include <sophus/se3.h>
#include <sophus/so3.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

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
                            Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                        );
        eigen2cv(Eigen::Matrix3d(T_c_w_estimated_.rotation_matrix()),R);
        cout<<"R!!!"<<endl;
        cout<<T_c_w_estimated_.rotation_matrix()<<endl;
        cout<<endl;
        
        cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵
        cout<<"R: "<<endl<<R<<endl;
        //use BA optimizer
        Eigen::Matrix4d Twc;
        bundleAdjustment(pts_3d,pts_2d,K,R,t,Twc,inliers);

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
                if(point->pt_2d_inframe[refframe->id_].x!=keypoints_1[good_matches[i].queryIdx].pt.x&&
                point->pt_2d_inframe[refframe->id_].y!=keypoints_1[good_matches[i].queryIdx].pt.y)
                {
                    Eigen::Vector4d homogeneous_pts_3d<<pts_3d[i].x,pts_3d[i].y,pts_3d[i].z,1;
                    Eigen::Vector4d world_homogeneous_pts_3d=Twc.inverse()*homogeneous_pts_3d;
                    
                    new_pts_3d.push_back(pts_3d[i]);
                    break;
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
        
        drawMatches ( refframe->color_, keypoints_1, curframe->color_, keypoints_2, good_matches, img_goodmatch );
        imshow ( "优化后匹配点对", img_goodmatch );
        drawKeypoints( curframe->color_, keypoints_1, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
        imshow("ORB特征点",outimg);
        waitKey(2);
    }

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
        vector<Point3d> Points3D;
        vector<Eigen::Matrix4d> Twc_;
        for(auto points:Map->map_points_)
        {
            Points3D.push_back(points.second->pos_);        
        }
        for(auto frame:Map->keyframes_)
        {
            Twc_.push_back(frame.second->T_c_w_);
        }
        DrawPoints(Points3D);
        DrawFrames(Twc_);
        
        pangolin::FinishFrame();

        usleep(500);
    }
 
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
