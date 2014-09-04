#!/bin/bash 

if [$# -eq 1] || [$2 !="opencv-2.4.9.zip"] ||[ $1 != "opencv-2.4.9.zip"];then
        echo "please use opencv-2.4.9.zip"
        echo "usage : installOpencv <clean> <opencv-2.4.9.zip>"
        exit 1
fi

sudo apt-get update
if [$1 = "clean"]; then
	echo "removing ffmpeg x24 .."
	sudo apt get remove ffmpeg x264
fi



echo "installing dependencies"

sudo apt-get install ocl-icd-libopencl1 build-essential checkinstall cmake pkg-config yasm libjpeg-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev python-dev python-numpy libtbb-dev libqt4-dev libgtk2.0-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils





mkdir ~/opencv
sudo unzip ~/opencv-2.4.9.zip -d  ~/opencv
cd ~/opencv/opencv-2.4.9
mkdir build
cd ./build

echo "now running cmake" 
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON ..

echo "now runnin make"
sudo make -j8
sudo make install

sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig

echo "Done!" 
echo "credits .. Akshaya ;)"
echo "Edit etc/bash.bashrc !! : add -> PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig export PKG_CONFIG_PATH "
echo "try running example \n cd /usr/local/share/OpenCV/samples/cpp
g++ houghlines.cpp -o application `pkg-config --cflags --libs opencv`
./application"


