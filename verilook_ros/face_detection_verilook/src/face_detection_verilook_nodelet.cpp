/**/

#include <face_detection_verilook/face_detection_verilook_node.h>

/* ROS */
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace face_detection_verilook
{
    class FaceDetectionVerilookNodelet : public nodelet::Nodelet
    {
    protected:
        ros::NodeHandle node_handle_;
        FaceDetectionVerilookNode * face_detection_node_;

    public:
        FaceDetectionVerilookNodelet(void)
        {
            face_detection_node_ = 0;
        }

        ~FaceDetectionVerilookNodelet(void)
        {
            if (face_detection_node_ != 0)
                delete face_detection_node_;
        }

        virtual void onInit(void)
        {
            node_handle_ = getNodeHandle();
            face_detection_node_ = new FaceDetectionVerilookNode(node_handle_);
        }
    };
}
// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(
    face_detection_verilook, FaceDetectionVerilookNodelet,
    face_detection_verilook::FaceDetectionVerilookNodelet, nodelet::Nodelet
)