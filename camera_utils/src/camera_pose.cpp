#include <iostream>
#include <cmath>
#include <aruco/aruco.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
void image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        exit(-1);
    }
    aruco::CameraParameters camera;
    camera.readFromXMLFile("/home/fetch/camera_utils/src/camera_utils/config/fetch_camera_calib.yml");
    aruco::MarkerMap mmap;
    mmap.readFromFile("/home/fetch/camera_utils/src/camera_utils/config/Configuration_meter.yml");
    aruco::MarkerMapPoseTracker MMTracker;
    MMTracker.setParams(camera,mmap);

    aruco::MarkerDetector Detector;
    Detector.setDictionary("ARUCO_MIP_36h12");
    auto markers=Detector.detect(cv_ptr->image);//0.05 is the marker size

    MMTracker.estimatePose(markers);
    cv::Mat tvec;
    cv::Mat rvec;
    if (MMTracker.isValid())
    {
        tvec = MMTracker.getTvec();
        rvec = MMTracker.getRvec();
        std::cout << "rvec: " << rvec << "; tvec: " << tvec << std::endl;
    }
    // getvec returns a 1 by 3 opencv mat
    if (rvec.size().width == 3)
    {
        // cv::Mat rot_matrix(3,3,CV_32FC1);
        // cv::Rodrigues(MMTracker.getRvec(), rot_matrix);
        // std::cout << rot_matrix << std::endl;

        tf::Transform transform;

        // set transform translation
        transform.setOrigin(tf::Vector3(tvec.at<float>(0,0), tvec.at<float>(0,1), tvec.at<float>(0,2)));
        
        // get rotation angle
        float theta = cv::norm(rvec);
        std::cout << "theta: " << theta << std::endl;

        // get rotation axis
        rvec /= theta;
        tf::Vector3 axis(rvec.at<float>(0,0), rvec.at<float>(0,1), rvec.at<float>(0,2));
        std::cout << "axis: " << rvec << std::endl;

        tf::Quaternion q(axis, theta);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "marker_map_frame", "dewey_camera"));
    }
    // cv::imshow("image",cv_ptr->image);
    // cv::waitKey(0);
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "local_camera_pose_estimator");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("head_camera/rgb/image_raw", 1, image_callback);
    ros::spin();
    return 0;
}
