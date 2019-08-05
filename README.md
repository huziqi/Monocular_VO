Monocular_VO
==========
Prepare for your environment
----------
Next few steps will help you install some essential packages and set up your environment.  
__Dependence components__:
* GCC 4.4.x or later
* CMake 2.6 or higher(3.2 better)
* GTK+2.x or higher, including headers (libgtk2.0-dev)
* pkg-config
* Python 2.6 or later and Numpy 1.5 or later with developer packages (python-dev, python-numpy)
* ffmpeg or libav development packages: libavcodec-dev, libavformat-dev, libswscale-dev
* Git
* [optional] libtbb2 libtbb-dev
* [optional] libdc1394 2.x
* [optional] libjpeg-dev, libpng-dev, libtiff-dev, libjasper-dev, libdc1394-22-dev  

__Packages__:  

    sudo apt-get install build-essential
    sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev
    sudo apt-get install libjasper-dev libdc1394-22-dev #processing image
    sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev liblapacke-dev
    sudo apt-get install libxvidcore-dev libx264-dev #processing video
    sudo apt-get install libatlas-base-dev gfortran #optimize opencv
    sudo apt-get install ffmpeg
    sudo apt-get install libvtk5-dev #libVTK
    sudo apt-get install libboost-dev
  
__download OpenCV 3.2.0__  

    wget https://github.com/opencv/opencv/archive/3.2.0.zip  
    
Extract the zip file and excute follow commands:  

    cd opencv-3.2.0
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DWITH_VTK=ON ..\
    make -j8
    sudo make install
    sudo ldconfig  
    
__Eigen__  

    wget https://bitbucket.org/eigen/eigen/get/3.2.10.tar.gz
    tar zxvf eigen-3.2.10.tar.gz  
__DBoW2__  

    sudo apt-get install libboost-dev
    git clone https://github.com/dorian3d/DBoW2  
__OpenGL__  
Details can refer to https://blog.csdn.net/l297969586/article/details/53534807  

    sudo apt-get install build-essential  
    sudo apt-get install libgl1-mesa-dev  
    sudo apt-get install libglu1-mesa-dev  
    sudo apt-get install freeglut3-dev  
__Pangolin__

    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    mkdir build
    cd build
    cmake ..
    cmake --build .  
__other__

    sudo apt-get install libglew-dev
    sudo apt-get install libpython2.7-dev  
    
__Test on ORB-SLAM2__

    git clone https://github.com/raulmur/ORB_SLAM2.git
    cd ORB_SLAM2
    chmod +x build.sh
    ./build.sh
    ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER  
Test data from http://vision.in.tum.de/data/datasets/rgbd-dataset/download  

How to complie  
----------  
Make sure you have already settled your environment following above instrction. The tutorial for Monocular VO can also refer to:  
* https://gitee.com/paopaoslam/ORB-SLAM2
* http://www.fengbing.net/  
About C++ grammer can refer to:  
* https://zh.cppreference.com/w/  
Git clone to your workspace,and excute:   

    cd ~/Monocular_vo/
    mkdir build
    cd build
    cmake ..
    make
    
