//
// Created by zhj on 19-7-21.
//
#include <pangolin/pangolin.h>
#include <iostream>
#include <vector>


const int window_size = 2;	// NCC 取的窗口半宽度
const int ncc_area = (2*window_size+1)*(2*window_size+1); // NCC窗口面积

inline float find_min(float conf[])
{
    float result=1000000000;
    for(int i=0;i<5;i++)
    {
        if(conf[i]<result&&conf[i]!=-1)
        {
            result=conf[i];
        }
    }
    return result;
}

void DrawPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    glPointSize(1.0f);
    glBegin(GL_POINTS);
    for(size_t i = 0;i<cloud->size();++i){
        glColor3f(float(cloud->points[i].r)/255,float( cloud->points[i].g)/255,float(cloud->points[i].b)/255);
        glVertex3d(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
    }
    glEnd();
}

void DrawFrames(std::vector<Sophus::SE3> Twc){
    const float w = 0.01;
    const float h = w * 0.75f;
    const float z = w * 0.6f;

    for(int i=0;i<Twc.size();i++)
    {
        
        glPushMatrix();
        glMultMatrixd(Twc[i].matrix().data());
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

inline bool inImage(const mydense::Frame frame,const Eigen::Vector2d p)
{
    return p[0]>boarder&&p[1]>boarder&&p[0]<frame.rgb.cols-boarder&&p[1]<frame.rgb.rows-boarder;
}

float RGB_SSD(const cv::Mat cur,const cv::Mat ref,Eigen::Vector2d p_cur,Eigen::Vector2d p_ref)
{
    float result=0;
    for(int x=-window_size;x<=window_size;x++)
    {
        for(int y=-window_size;y<=window_size;y++)
        {
            //
            float b=(cur.data[(int(p_cur[1]+y))*cur.step+(int(p_cur[0]+x))*cur.channels()]-ref.data[(int(p_ref[1]+y))*ref.step+(int(p_ref[0]+x))*ref.channels()]);
            float g=(cur.data[(int(p_cur[1]+y))*cur.step+(int(p_cur[0]+x))*cur.channels()+1]-ref.data[(int(p_ref[1]+y))*ref.step+(int(p_ref[0]+x))*ref.channels()+1]);
            float r=(cur.data[(int(p_cur[1]+y))*cur.step+(int(p_cur[0]+x))*cur.channels()+2]-ref.data[(int(p_ref[1]+y))*ref.step+(int(p_ref[0]+x))*ref.channels()+2]);
            result=result+b*b+g*g+r*r;
            //cout<<"b: "<<b<<" g: "<<g<<" r: "<<r<<endl;
        }
    }
    //cout<<"result:"<<result<<endl;
    //if(result<0.001)
        //exit(1);
        //cv::waitKey(0);
        //cout<<"------------------------------------!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
    //else
    return result;
}

bool readPose(std::string path,std::vector<Sophus::SE3> &pose_T)
{
    ifstream fin(path);
    if(!fin) return false;
    pose_T.resize(0);

    while(!fin.eof())
    {
        int image_id;
        double data[7];
        std::string image_name;
        fin>>image_id;
        for(double &d:data) fin>>d;
        fin>>image_id>>image_name;
        pose_T.push_back(Sophus::SE3(Eigen::Quaterniond(data[0], data[1], data[2], data[3]),
                                     Eigen::Vector3d(data[4], data[5], data[6])));
        if(!fin.good()) break;
    }
    return true;
}

bool readDatasetFiles(const std::string& path, std::vector< std::string >& color_image_files, std::vector<Sophus::SE3>& poses)
{
    ifstream fin( path+"/first_200_frames_traj_over_table_input_sequence.txt");
    if ( !fin ) return false;

    while ( !fin.eof() )
    {
        // 数据格式：图像文件名 tx, ty, tz, qx, qy, qz, qw ，注意是 TWC 而非 TCW
        string image;
        fin>>image;
        double data[7];
        for ( double& d:data ) fin>>d;

        color_image_files.push_back( path+string("/images/")+image );
        poses.push_back(
                Sophus::SE3(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                     Eigen::Vector3d(data[0], data[1], data[2]))
        );
        if ( !fin.good() ) break;
    }
    return true;
}

std::vector<cv::Mat> GenerateConfidence(const std::vector<mydense::Frame> frames)
{
    vector<cv::Mat> confs;
    for(int i=0;i<frames.size()-1;i++)
    {
        cv::Mat confidence(frames[i].rgb.size(),CV_64FC1,-1);
        // 选取boarder 进行窗口测试
        for(int v=boarder;v<frames[i].rgb.rows-boarder;v++)
        {
            for(int u=boarder;u<frames[i].rgb.cols-boarder;u++)
            {
                // check point
                double d=frames[i].depth.ptr<double>(v)[u]*255;
                //cout<<"depth from colmap: "<<d<<endl;
                if(d<=0||d>150)
                {
                    continue;
                }
                // 当前帧的 深度邻域
                float conf[5]={-1,-1,-1,-1,-1,};
                float scale=0.8;
                for(int scale_index=0;scale_index<5;scale_index++)
                {
                    // 点的重投影： 当前帧2d->camera 3d->相邻帧2d
                    Eigen::Vector3d pc;
                    //pc[2]=d*scale;
                    pc[2]=d;
                    pc[1]=(v-cy)*pc[2]/fy;
                    pc[0]=(u-cx)*pc[2]/fx;
                    Eigen::Vector3d pwc_3d=frames[i].pose.inverse()*pc; // P_W
                    //cout<<"PWC: "<<pwc_3d[0]<<"  "<<pwc_3d[1]<<"  "<<pwc_3d[2]<<endl;
                    Eigen::Vector3d pr=(frames[i].pose.inverse()*frames[i+step].pose)*pc;
                    Eigen::Quaterniond q=Eigen::Quaterniond(frames[i].pose.rotation_matrix());
//                    cout<<"pose i : "<<q.w()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<endl;
//                    cout<<"pose i+step : "<<frames[i+step].pose<<endl;
                    // T_RC=T_RW*T_WC
                    //cout<<"d: "<<d<<"  sacel:"<<scale<<"  pc[2]"<<pc[2]<<"  "<<endl;
                    //cout<<"PC: "<<pc.transpose();
                    //cout<<"PR: "<<pr.transpose();
                    //cout<<"SE3: "<<frames[i+1].pose<<"   "<<frames[i].pose<<endl;
                    //cout<<"R_T: "<<frames[i+30].pose<<" "<<frames[i].pose<<endl;
                    //cout<<"R_T: "<<(frames[i+step].pose.inverse()*frames[i].pose).matrix()<<endl;

                    Eigen::Vector2d pr_pixel;
                    pr_pixel[0]=(pr[0]/pr[2])*fx+cx;
                    pr_pixel[1]=(pr[1]/pr[2])*fy+cy;
                    scale+=0.1;
                    //cout<<"updaet scale :"<<scale<<endl;
                    //cout<<"   points:"<<pr_pixel[0]<<"   "<<pr_pixel[1]<<"  "<<flush;


                    // boarder condition
                    if(!inImage(frames[i+step],pr_pixel))
                    {
                        cout<<"qweqwdre"<<endl;
                        continue;
                    }

                    cv::Mat rgb_l,rgb_r;
                    rgb_l=frames[i].rgb.clone();
                    rgb_r=frames[i+step].rgb.clone();
                    cv::circle( rgb_l, cv::Point2f(int(u), int(v)), 5, cv::Scalar(0,0,250), 2);
                    cv::circle( rgb_r, cv::Point2f(int(pr_pixel[0]), int(pr_pixel[1])), 5, cv::Scalar(0,0,250), 2);
                    cv::imshow("rgb_l",rgb_l);
                    cv::imshow("rgb_r",rgb_r);
                    cv::waitKey(5);
                    // project to reference frame  compute SSD
                    //conf[scale_index]=RGB_SSD(frames[i].rgb,frames[i+1].rgb,Eigen::Vector2d(u,v),pr_pixel);
                    cout<<RGB_SSD(frames[i].rgb,frames[i+step].rgb,Eigen::Vector2d(u,v),pr_pixel)<<endl;

                }
                float min_ssd=find_min(conf);
                //cout<<"min_ssd: "<<min_ssd<<"  sample: "<<conf[0]<<"     "<<conf[1]<<"     "<<conf[2]<<"   "<<conf[3]<<"    "<<conf[4]<<endl;
                confidence.ptr<double>(v)[u]=min_ssd/conf[2];
                //cout<<"confidence: "<<min_ssd/conf[2]<<endl;
            }
        }
        confs.push_back(confidence);
    }
    return confs;
}


cv::Mat Generatemask(const mydense::Frame frame)
{
    // 类型转换
    cv::Mat depth=frame.depth.clone();
    depth.convertTo(depth,CV_8UC1,255);
    cv::imshow("img",depth);

    // compute grad
    cv::Mat grad_x,grad_y,abs_gard_x,abs_grad_y;
    cv::Mat dst;
    cv::Sobel(depth,grad_x,CV_16S,1,0,3,1,1,cv::BORDER_DEFAULT);
    cout<<"soel type:"<<grad_x.type()<<endl;
    cv::Sobel(depth,grad_y,CV_16S,0,1,3,1,1,cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x,abs_gard_x);
    cv::convertScaleAbs(grad_y,abs_grad_y);
    cv::addWeighted(abs_gard_x,0.5,abs_grad_y,0.5,0,dst);
    cv::imshow("Grad",dst);
    cout<<"type:"<<dst.type()<<endl;


    // generate mask
    int zz=dst.ptr<uchar>(12)[12];
    cout<<"zzz"<<zz<<endl;

    cv::Mat mask(depth.size(),CV_8UC1,cv::Scalar(0));
    for(int i=0;i<depth.rows;)
    {
        for(int j=0;j<depth.cols;j)
        {
            int z=int(dst.ptr<uchar>(i)[j])/20;
            int j_step=0;
            switch (z)
            {
                case 0:
                    j_step=7;
                    break;
                case 1:
                    j_step=6;
                    break;
                case 2:
                    j_step=5;
                    break;
                case 3:
                    j_step=4;
                    break;
                case 4:
                    j_step=3;
                    break;
                default:
                    j_step=2;
            }

            mask.ptr<uchar>(i)[j]=255;
            j+=j_step;
        }
        i+=2;
    }
    //
    cv::imshow("mask",mask);
    cv::Mat result;
    depth.copyTo(result,mask);
    cv::imshow("result",result);
    return result;
}

int main()
{
    // load file
    std::vector<Sophus::SE3> poses_TCW;
    



    // Pangolin for visualization
    pangolin::CreateWindowAndBind("3D Reconstruction", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 0, -10, 0, 0, 0, pangolin::AxisNegY)
    );


    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    //
    // load: rgb->depth->compute mask->compute confidence
    // load RGB + depth
    // compute PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<mydense::Frame> frames;
    frames.clear();
    int real_i=-1;
    for(int i=0;i<200;i++)
    {
        // read RGB_IMAGE
        cv::Mat rgb=cv::imread((rgbfmt%i).str());
        if(!rgb.data)
        {
            //cout<<"Can not read rgb image!"<<(rgbfmt%i).str()<<endl;
            continue;
        }
        real_i++;
        cout<<"Read rgb image!"<<(rgbfmt%i).str()<<endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur(new pcl::PointCloud<pcl::PointXYZRGB>);
        mydense::Frame frame(real_i);
        Sophus::SE3 pose=poses_TCW[real_i];

        // read DepthMap in Colmap format
        colmap::mvs::DepthMap img;
        img.Read((depfmt%i).str());
        float d;
        cv::Mat depth(rgb.size(),CV_64FC1,-1);
        for (size_t ii = boarder; ii <rgb.rows-boarder ; ii++) {
            for (size_t j = boarder; j < rgb.cols-boarder; j++) {
                d = img.Get(ii, j);
                if(d<=0||d>150)
                    continue;
                depth.ptr<double>(ii)[j]=double(d)/255; // 深度值
                Eigen::Vector3d point;
                point[2]=d/40;
                point[1]=(ii-cy)*point[2]/fy;
                point[0]=(j-cx)*point[2]/fx;
                Eigen::Vector3d p3d=poses_TCW[real_i]*point;

                pcl::PointXYZRGB p;
                p.x=float(p3d[0]);
                p.y=float(p3d[1]);
                p.z=float(p3d[2]);
                p.b=rgb.data[ii*rgb.step+j*rgb.channels()];
                p.g=rgb.data[ii*rgb.step+j*rgb.channels()+1];
                p.r=rgb.data[ii*rgb.step+j*rgb.channels()+2];
                cur->points.push_back(p);
                //cout<<"p3d:   "<<p.x<<"  "<<p.y<<"  "<<p.z<<endl;
            }
        }
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp ( new pcl::PointCloud<pcl::PointXYZRGB>);
//         pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
//         statistical_filter.setMeanK(50);
//         statistical_filter.setStddevMulThresh(1.0);
//         statistical_filter.setInputCloud(cur);
//         statistical_filter.filter( *tmp );
//         (*cloud) += *tmp;
         (*cloud) += *cur;
        frame.depth=depth;
        frame.pose=pose;
        frame.rgb=rgb;

        // sample: mask
        //frame.mask=Generatemask(frame);

        frames.push_back(frame);
    }

    // depth fusion -> generate confidence
    GenerateConfidence(frames);
    //

    cloud->is_dense = false;
    cout<<"Point Cloud size:"<<cloud->size()<<endl;
    // voxel filter
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setLeafSize( 0.01, 0.01, 0.01 );       // resolution
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp ( new pcl::PointCloud<pcl::PointXYZRGB> );
    voxel_filter.setInputCloud( cloud );
    voxel_filter.filter( *tmp );
    tmp->swap( *cloud );
    cout<<"滤波之后，点云共有："<<cloud->size()<<endl;

#ifdef PANGOLIN_DEBUG
    while(1)
    {
        // draw 3d points + camera trajec
        d_cam.Activate(s_cam);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        DrawFrames(poses_TWC);
        DrawPoints(cloud);
        //waitKey(0);
        pangolin::FinishFrame();
    }
#endif

    cv::waitKey(0);
    return 0;
}







// #include<pangolin/pangolin.h>
// #include<mutex>


// void View(Map *map) {
//     pangolin::CreateWindowAndBind("SfM: Map Viewer", 1024, 768);
//     glEnable(GL_DEPTH_TEST);
//     glEnable(GL_BLEND);
//     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//     pangolin::OpenGlRenderState s_cam(
//             pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
//             pangolin::ModelViewLookAt(0, 0, -10, 0, 0, 0, pangolin::AxisNegY)
//     );

//     pangolin::MyHandler3D myHandler3D(s_cam, map);

//     pangolin::View &d_cam = pangolin::CreateDisplay()
//             .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
//             .SetHandler(&myHandler3D);

//     stable.resize((size_t)map->m_tracks.size(),false);
//     for(size_t i = 0;i<map->m_tracks.size();++i){
//         auto &track = map->m_tracks[i];
//         if (track.invalid || !track.triangulated||track.outlier)continue;
//         int count = 0;
//         for(auto&obs:track.m_observations){
//             auto&frame = map->m_frames[obs.first];
//             if(frame.registered)
//                 count++;
//         }
//         if(count>0)
//             stable[i] = true;
//     }

//     while (map->m_tracks.size() > 0) {
//         d_cam.Activate(s_cam);
//         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//         glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

//         DrawPoints(map);
//         DrawFrames(map);

//         pangolin::FinishFrame();

//         usleep(500);
//     }
// }

// void DrawFrames(Map *map,int select_id=-1){
//     const float w = 0.01;
//     const float h = w * 0.75f;
//     const float z = w * 0.6f;
//     for (const auto &frame:map->m_frames) {
//         if (!frame.registered)continue;

//         glPushMatrix();
//         auto Twc = frame.twc();
//         glMultMatrixd(Twc.data());

//         double red = (frame.sre - 0.25) / 0.5;
//         glColor3d(red, (1 - red), 0);

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


// void DrawPoints(Map *map){
//     glPointSize(1.0f);
//     glBegin(GL_POINTS);
//     for(size_t i = 0;i<map->m_tracks.size();++i){
//         auto &track = map->m_tracks[i];
// 	glColor3f(1.0f, 0.0f, 0.0f);
//         const auto &p = track.m_point3d;
//         glVertex3d(p(0), p(1), p(2));
//     }
//     glEnd();
// }