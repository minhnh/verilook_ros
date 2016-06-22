/* Copyright 2016 Bonn-Rhein-Sieg University
 * Author: Minh Nguyen
 */
#include <face_recognition_verilook_node.h>
#include <string>

// Nodelet
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

/* Local */

namespace verilook_ros
{

class FaceRecognitionVerilookNodelet : public nodelet::Nodelet
{
public:
    FaceRecognitionVerilookNodelet()
    {
        ROS_INFO_STREAM(PACKAGE_NAME << ": face detection nodelet starting...");
        face_recognition_node_ = 0;
    }
    virtual ~FaceRecognitionVerilookNodelet()
    {
        if (face_recognition_node_ != 0)
            delete face_recognition_node_;
    }

private:
    virtual void onInit()
    {
        node_handle_ = getMTNodeHandle();
        face_recognition_node_ = new verilook_ros::FaceRecognitionVerilookNode(node_handle_);
    }

protected:
    ros::NodeHandle node_handle_;
    verilook_ros::FaceRecognitionVerilookNode * face_recognition_node_;

};

PLUGINLIB_EXPORT_CLASS(verilook_ros::FaceRecognitionVerilookNodelet, nodelet::Nodelet);

}   // namespace verilook_ros
