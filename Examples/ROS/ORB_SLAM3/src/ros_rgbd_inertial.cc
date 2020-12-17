/**
* This file is part of ORB-SLAM3
*
* Copydepth (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copydepth (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bRect, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe){}

    void GrabImageRgb(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageDepth(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgRgbBuf, imgDepthBuf;
    std::mutex mBufMutexRgb,mBufMutexDepth;

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};


int main(int argc, char **argv){
     ros::init(argc, argv, "RGBD_Inertial");
     ros::NodeHandle nh;
     ros::NodeHandle pnh("~");
     ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

     bool doClahe, doRect, visualize;
     std::string vocab_path, settings_path;
     pnh.param<std::string>("vocab_path", vocab_path, "/home/hunter/libraries/ORB_SLAM3/Vocabulary/ORBvoc.txt");
     pnh.param<std::string>("calib_path", settings_path, "/home/hunter/devel/ORB_SLAM3/Examples/ROS/ORB_SLAM3/params/d415_imu.yaml");
     pnh.param<bool>("rectify", doRect, false);
     pnh.param<bool>("equalize", doClahe, false);
     pnh.param<bool>("visualize", visualize, true);

     // Create SLAM system. It initializes all system threads and gets ready to process frames.
     ORB_SLAM3::System SLAM(vocab_path, settings_path, ORB_SLAM3::System::IMU_RGBD, visualize);
     ImuGrabber imugb;
     ImageGrabber igb(&SLAM, &imugb, doRect, doClahe);

     std::string rgb_topic, depth_topic, imu_topic;
     pnh.param<std::string>("rgb_topic", rgb_topic, "camera/color/image_raw");
     pnh.param<std::string>("depth_topic", depth_topic, "camera/depth/image_rect_raw");
     pnh.param<std::string>("imu_topic", imu_topic, "imu");

     // Maximum delay, 5 seconds
     ros::Subscriber sub_imu = nh.subscribe(imu_topic, 1000, &ImuGrabber::GrabImu, &imugb);
     ros::Subscriber sub_img_rgb = nh.subscribe(rgb_topic, 100, &ImageGrabber::GrabImageRgb,&igb);
     ros::Subscriber sub_img_depth = nh.subscribe(depth_topic, 100, &ImageGrabber::GrabImageDepth,&igb);

     std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);
     ros::spin();
     return 0;
}

void ImageGrabber::GrabImageRgb(const sensor_msgs::ImageConstPtr &img_msg){
     mBufMutexRgb.lock();
     if (!imgRgbBuf.empty()) imgRgbBuf.pop();
     imgRgbBuf.push(img_msg);
     mBufMutexRgb.unlock();
}
void ImageGrabber::GrabImageDepth(const sensor_msgs::ImageConstPtr &img_msg){
     mBufMutexDepth.lock();
     if (!imgDepthBuf.empty()) imgDepthBuf.pop();
     imgDepthBuf.push(img_msg);
     mBufMutexDepth.unlock();
}
cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg){
     // Copy the ros image message to cv::Mat.
     cv_bridge::CvImageConstPtr cv_ptr;
     try{ cv_ptr = cv_bridge::toCvShare(img_msg);
     } catch (cv_bridge::Exception& e){ ROS_ERROR("cv_bridge exception: %s", e.what()); }
     return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu(){
     const double maxTimeDiff = 0.5;
     double tStart = ros::Time::now().toSec();
     while(1){
          int count = 0;
          cv::Mat imRgb, imDepth;
          double tImRgb = 0, tImDepth = 0;
          if( !imgRgbBuf.empty() &&! imgDepthBuf.empty() && !mpImuGb->imuBuf.empty()){
               tImRgb = imgRgbBuf.front()->header.stamp.toSec();
               tImDepth = imgDepthBuf.front()->header.stamp.toSec();

               this->mBufMutexDepth.lock();
               while((tImRgb-tImDepth)>maxTimeDiff && imgDepthBuf.size()>1){
                    imgDepthBuf.pop();
                    tImDepth = imgDepthBuf.front()->header.stamp.toSec();
               }
               this->mBufMutexDepth.unlock();

               this->mBufMutexRgb.lock();
               while((tImDepth-tImRgb)>maxTimeDiff && imgRgbBuf.size()>1){
                    imgRgbBuf.pop();
                    tImRgb = imgRgbBuf.front()->header.stamp.toSec();
               }
               this->mBufMutexRgb.unlock();

               if((tImRgb-tImDepth)>maxTimeDiff || (tImDepth-tImRgb)>maxTimeDiff){
                    ROS_WARN_THROTTLE(1, "RGB and Depth images not in tolerance --- tColor, tDepth, delta (secs) ---- %.3lf, %.3lf, %.3lf",
                         tImRgb-tStart, tImDepth-tStart, tImDepth-tImRgb
                    );
                    continue;
               }

               if(tImRgb > mpImuGb->imuBuf.back()->header.stamp.toSec()){
                    ROS_WARN_THROTTLE(1, "Oldest RGB stamp > current IMU Stamp --- tColor, tDepth, tImu (secs) ---- %.3lf, %.3lf, %.3lf",
                         tImRgb-tStart, tImDepth-tStart, mpImuGb->imuBuf.back()->header.stamp.toSec()-tStart
                    );
                    continue;
               }

               this->mBufMutexRgb.lock();
               imRgb = GetImage(imgRgbBuf.front());
               imgRgbBuf.pop();
               this->mBufMutexRgb.unlock();
               this->mBufMutexDepth.lock();
               imDepth = GetImage(imgDepthBuf.front());
               imgDepthBuf.pop();
               this->mBufMutexDepth.unlock();

               vector<ORB_SLAM3::IMU::Point> vImuMeas;
               mpImuGb->mBufMutex.lock();

               if(!mpImuGb->imuBuf.empty()){
                    // Load imu measurements from buffer
                    vImuMeas.clear();
                    while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImRgb){
                         count++;
                         double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                         ROS_DEBUG("Adding IMU point #%d ---- tColor, tDepth, tImu (secs) ---- %.3lf, %.3lf, %.3lf",
                              count, tImRgb-tStart, tImDepth-tStart, t-tStart
                         );

                         cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                         cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                         vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                         mpImuGb->imuBuf.pop();
                    }
               }
               mpImuGb->mBufMutex.unlock();

               if(mbClahe){
                    mClahe->apply(imRgb,imRgb);
                    mClahe->apply(imDepth,imDepth);
               }
               mpSLAM->TrackRGBD(imRgb,imDepth,tImRgb,vImuMeas);

               std::chrono::milliseconds tSleep(1);
               std::this_thread::sleep_for(tSleep);
          }
     }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg){
     mBufMutex.lock();
     imuBuf.push(imu_msg);
     mBufMutex.unlock();
     return;
}
