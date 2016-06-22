/* Copyright 2016 Bonn-Rhein-Sieg University
 * Author: Minh Nguyen
 */
#ifndef VERILOOK_ROS_FACE_DETECTION_VERILOOK_NODE_H
#define VERILOOK_ROS_FACE_DETECTION_VERILOOK_NODE_H

/* System */
#include <boost/thread.hpp>

/* ROS */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

/* Neurotec */
#include <Images/NImage.hpp>
#include <NBiometricClient.hpp>

/* Package */
#include "verilook_ros/CreateTemplate.h"
#include "verilook_wrapper.h"

namespace verilook_ros
{

#define PACKAGE_NAME    "verilook_ros"

/* Forward declarations */
class VerilookWrapper;

class FaceRecognitionVerilookNode
{
public:
    FaceRecognitionVerilookNode(ros::NodeHandle nh);
    ~FaceRecognitionVerilookNode();
    void getImage(Neurotec::Images::HNImage *phImage);

private:
    void eventInCallback(const std_msgs::String::Ptr &msg);
    void subjectIDCallback(const std_msgs::String::Ptr &msg);
    bool condFulfilled();
    void imageMessageCallback(const sensor_msgs::Image::ConstPtr& msg);
    bool createTemplateServiceCallback(CreateTemplate::Request& request, CreateTemplate::Response& response);

    Neurotec::Images::HNImage image_buffer = NULL;
    Neurotec::Biometrics::Client::NBiometricClient m_biometricClient;
    VerilookWrapper * m_verilookWrapper;

    ros::Publisher pub_event_out_;
    ros::Subscriber m_sub_eventIn;
    ros::Subscriber m_sub_subjectID;
    image_transport::Subscriber image_sub;
    sensor_msgs::ImageConstPtr mp_image;

    boost::mutex mtx;
    boost::condition_variable cond;

};

}   // namespace verilook_ros

#endif  // VERILOOK_ROS_FACE_DETECTION_VERILOOK_NODE_H
