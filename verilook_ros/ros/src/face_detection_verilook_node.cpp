/* Copyright 2016 Bonn-Rhein-Sieg University
 * Author: Minh Nguyen
 *
 * Based on port from https://github.com/hansonrobotics/ros_verilook
 */

/* ROS */

/* Package */
#include <verilook_wrapper.h>
#include <face_detection_verilook_node.h>

namespace verilook_ros
{

FaceDetectionVerilookNode::FaceDetectionVerilookNode(ros::NodeHandle nh)
{
    // Obtain VeriLook license
    obtainVerilookLicenses();
    // create biometric client
    biometricClient.SetBiometricTypes(Neurotec::Biometrics::nbtFace);
    biometricClient.Initialize();

    // Start camera service
//    ros::ServiceServer service = nh.advertiseService(
//            "create_face_template", &FaceDetectionVerilookNode::createTemplateServiceCallback, this);

    // event publisher and subscriber
    pub_event_out_ = nh.advertise<std_msgs::String>("event_out", 1);
    sub_event_in_ = nh.subscribe("event_in", 1, &FaceDetectionVerilookNode::eventInCallback, this);

}

FaceDetectionVerilookNode::~FaceDetectionVerilookNode()
{
    releaseVerilookLicenses();
    if (!biometricClient.IsNull())
    {
        biometricClient.Cancel();
    }
    NCore::OnExit(false);
}

// Put incoming sensor_msgs/Image messages to a buffer
void FaceDetectionVerilookNode::imageMessageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    using Neurotec::Images::NImageCreateFromDataEx;
    using Neurotec::Images::NPF_RGB_8U;

    boost::lock_guard<boost::mutex> lock(mtx);

    ROS_INFO_STREAM(PACKAGE_NAME << ": image message callback");
    // Create the Image object
    HNImage newImage = NULL;
    NResult result = NImageCreateFromDataEx(NPF_RGB_8U.GetValue(), msg->width, msg->height,
                                            msg->step, msg->step, &msg->data[0],
                                            msg->height*msg->step, 0, &newImage);
    if (NFailed(result))
    {
        result = printErrorMsgWithLastError("NImageCreateWrapperEx() failed, result = %d\n", result);
    }
    else
    {
        // If buffer is not equal to NULL, free its memory
        NObjectSet(NULL, (HNObject*) &image_buffer);
        // Place the new image for grabs by the getImage function
        image_buffer = newImage;
    }

    cond.notify_one();

    // Shutdown the image stream to save CPU usage
    image_sub.shutdown();
}

bool FaceDetectionVerilookNode::condFulfilled()
{
    return image_buffer != NULL;
}

// Callback, from which EnrollFaceFromImageFunction gets its images.
void FaceDetectionVerilookNode::getImage(HNImage *phImage)
{
    boost::unique_lock<boost::mutex> lock(mtx);
    boost::system_time const timeout = boost::get_system_time() + boost::posix_time::seconds(1);
    if (cond.timed_wait(lock, timeout, boost::bind(&FaceDetectionVerilookNode::condFulfilled, this)))
    {
        *phImage = image_buffer;
        image_buffer = NULL;
    }
    else
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << "Timed out while waiting on ROS image stream.");
    }
}

bool FaceDetectionVerilookNode::createTemplateServiceCallback(
        CreateTemplate::Request& request,
        CreateTemplate::Response& response)
{
    ROS_INFO_STREAM(PACKAGE_NAME << "create template request: " << request.output_filename.c_str());

    // Free a possible leftover frame from last service call.
    NObjectSet(NULL, (HNObject*) &image_buffer);

    // Subscribe to the image stream
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(
            "/usb_cam/image_raw", 10, &FaceDetectionVerilookNode::imageMessageCallback, this);

    // Invoke the main big "create template" or "enroll face" routine.
    NRect boundingRect;
    NResult result = enrollFaceFromImageFunction(request.output_filename,
                                                 &FaceDetectionVerilookNode::getImage,
                                                 this, &boundingRect, biometricClient);

    // Fill the service response
    if (NFailed(result))
    {
        response.success = false;
    }
    else
    {
        response.success = true;
        response.face_position.x_offset = boundingRect.X;
        response.face_position.y_offset = boundingRect.Y;
        response.face_position.width = boundingRect.Width;
        response.face_position.height = boundingRect.Height;
    }

    // Shutdown the image stream to save CPU usage
    sub.shutdown();
    return true;
}

void FaceDetectionVerilookNode::eventInCallback(const std_msgs::String::Ptr &msg)
{
    ROS_INFO_STREAM(PACKAGE_NAME << ": in event_in callback...");
    if (msg->data == "e_start")
    {
        // Subscribe to the image stream
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_sub = it.subscribe(
                "/usb_cam/image_raw", 10, &FaceDetectionVerilookNode::imageMessageCallback, this);

        // Invoke the main big "create template" or "enroll face" routine.
        NRect boundingRect;
        NResult result = Neurotec::N_OK;
        result = enrollFaceFromImageFunction("/home/minh/.ros/data/verilook_ros/template_file",
                                                     &FaceDetectionVerilookNode::getImage,
                                                     this, &boundingRect, biometricClient);

        // Fill the service response
        if (NFailed(result))
        {
            //response.success = false;
            ROS_ERROR_STREAM(PACKAGE_NAME << ": enroll failed");
        }
        else
        {
            ROS_INFO_STREAM(PACKAGE_NAME << ": X = " << boundingRect.X << ", Y = " << boundingRect.Y);
        }

    }
}

}   // namespace verilook_ros

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "face_detection_verilook");

    ros::NodeHandle nh("~");

    verilook_ros::FaceDetectionVerilookNode face_detection_verilook_node(nh);

    // Start ROS node. We need at least two threads so that VeriLook can be
    // supplied with images in the middle of a service call.
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
