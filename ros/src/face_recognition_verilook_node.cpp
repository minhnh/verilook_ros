/* Copyright 2016 Bonn-Rhein-Sieg University
 * Author: Minh Nguyen
 */

/* ROS */

/* Package */
#include "face_recognition_verilook_node.h"
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

namespace verilook_ros
{

using Neurotec::Images::HNImage;
using Neurotec::Geometry::NRect;
using Neurotec::Biometrics::NBiometricTypes;

FaceRecognitionVerilookNode::FaceRecognitionVerilookNode(ros::NodeHandle nh)
: m_verilookWrapper(NULL), m_imageTransport(nh)
{
    try
    {
        NCore::OnStart();

        obtainVerilookLicenses();

        m_verilookWrapper = new VerilookWrapper(m_biometricClient);

    }
    catch (Neurotec::NError & e)
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << ": FaceDetectionVerilookNode initialization failed: " << std::string(e.GetMessage()));
    }

    // Start camera service
//    ros::ServiceServer service = nh.advertiseService(
//            "create_face_template", &FaceDetectionVerilookNode::createTemplateServiceCallback, this);

    m_imagePub = m_imageTransport.advertise("processed_image", 1);
    m_sub_eventIn = nh.subscribe("event_in", 1, &FaceRecognitionVerilookNode::eventInCallback, this);
    m_sub_subjectID = nh.subscribe("subject_id", 1, &FaceRecognitionVerilookNode::subjectIDCallback, this);

    pub_event_out_ = nh.advertise<std_msgs::String>("event_out", 1);

}

FaceRecognitionVerilookNode::~FaceRecognitionVerilookNode()
{
    releaseVerilookLicenses();
    if (!m_biometricClient.IsNull())
    {
        m_biometricClient.Cancel();
    }

    NCore::OnExit(false);

    delete m_verilookWrapper;
}

void FaceRecognitionVerilookNode::imageMessageCallback(const sensor_msgs::Image::ConstPtr& p_image)
{
    using Neurotec::Images::NImageCreateFromDataEx;
    using Neurotec::Images::NPF_RGB_8U;

    boost::lock_guard<boost::mutex> lock(mtx);

//    ROS_INFO_STREAM(PACKAGE_NAME << ": image message callback");
    // Create the Image object
    HNImage newImage = NULL;
    NResult result = NImageCreateFromDataEx(NPF_RGB_8U.GetValue(), p_image->width, p_image->height,
                                            p_image->step, p_image->step, &p_image->data[0],
                                            p_image->height*p_image->step, 0, &newImage);
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
        mp_image = p_image;
    }

    cond.notify_one();

    // Shutdown the image stream to save CPU usage
    m_imageSub.shutdown();
}

void FaceRecognitionVerilookNode::getImage(HNImage *phImage)
{
    boost::unique_lock<boost::mutex> lock(mtx);
    boost::system_time const timeout = boost::get_system_time() + boost::posix_time::seconds(1);
    if (cond.timed_wait(lock, timeout, boost::bind(&FaceRecognitionVerilookNode::condFulfilled, this)))
    {
        *phImage = image_buffer;
        image_buffer = NULL;
    }
    else
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << "Timed out while waiting on ROS image stream.");
    }
}

bool FaceRecognitionVerilookNode::condFulfilled()
{
    return image_buffer != NULL;
}

void FaceRecognitionVerilookNode::eventInCallback(const std_msgs::String::Ptr &msg)
{
//    ROS_INFO_STREAM(PACKAGE_NAME << ": in event_in callback...");
    // Subscribe to the image stream
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    m_imageSub = it.subscribe(
            "/usb_cam/image_raw", 10, &FaceRecognitionVerilookNode::imageMessageCallback, this);

    if (msg->data == "e_enroll")
    {
        m_verilookWrapper->enroll(&FaceRecognitionVerilookNode::getImage, this);
    }
    else if (msg->data == "e_identify")
    {
        m_verilookWrapper->identify(&FaceRecognitionVerilookNode::getImage, this);
    }
    else if (msg->data == "e_createTemplate")
    {
        m_verilookWrapper->createTemplate(&FaceRecognitionVerilookNode::getImage, this);
    }
    else if (msg->data == "e_showTemplate")
    {
        saveProcessedImage();
    }
}

void FaceRecognitionVerilookNode::saveProcessedImage()
{
    cv_bridge::CvImagePtr cv_image;
    try
    {
        cv_image = cv_bridge::toCvCopy(mp_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &ex)
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << ": Failed to convert image");
        return;
    }

    std::vector<VerilookFace> faces = m_verilookWrapper->getCurrentFaces();
    if (faces.size() <= 0)
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << ": no face recorded");
        return;
    }

    for(std::vector<VerilookFace>::iterator p_face = faces.begin(); p_face != faces.end(); ++p_face) {
        Neurotec::Geometry::NRect boundingRect = p_face->m_attributes.GetBoundingRect();
//        ROS_INFO("%s: found face at (%d, %d), width = %d, height = %d",
//                PACKAGE_NAME, boundingRect.X, boundingRect.Y,
//                boundingRect.Width, boundingRect.Height);

        std::stringstream info;
        info << "ID: " << p_face->m_id;
//        info << ", gender: " << std::string(Neurotec::NEnum::ToString(
//                NBiometricTypes::NGenderNativeTypeOf(), p_face->m_attributes.GetGender()));
//        info << ", expression: " << std::string(Neurotec::NEnum::ToString(
//                NBiometricTypes::NLExpressionNativeTypeOf(), p_face->m_attributes.GetExpression()));

        cv::Point2f pointTopLeft(boundingRect.X, boundingRect.Y);
        cv::Point2f pointBottomRight(boundingRect.X + boundingRect.Width, boundingRect.Y + boundingRect.Height);

        cv::RNG rng(12345);
        cv::Scalar colour(rng.uniform(125, 255), rng.uniform(125, 255), rng.uniform(125, 255));
        cv::Scalar colourDark(rng.uniform(0, 125), rng.uniform(0, 125), rng.uniform(0, 125));

        cv::rectangle(cv_image->image, pointTopLeft, pointBottomRight, colour, 2, 8, 0);

        cv::Point2f pointText(boundingRect.X, boundingRect.Y - 5);
        cv::putText(cv_image->image, info.str().c_str(), pointText,
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, colourDark, 1, CV_AA);
    }

    m_imagePub.publish(cv_image->toImageMsg());
}

void FaceRecognitionVerilookNode::subjectIDCallback(const std_msgs::String::Ptr &msg)
{
    if (msg->data.empty())
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << ": received empty subject ID");
    }
    else
    {
        ROS_INFO_STREAM(PACKAGE_NAME << ": received subject ID: " << msg->data);
        m_verilookWrapper->setSubjectID(msg->data);
    }
}

bool FaceRecognitionVerilookNode::createTemplateServiceCallback(
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
            "/usb_cam/image_raw", 10, &FaceRecognitionVerilookNode::imageMessageCallback, this);

    // Invoke the main big "create template" or "enroll face" routine.
    NRect boundingRect;
    NResult result = Neurotec::N_OK;
    m_verilookWrapper->enroll(&FaceRecognitionVerilookNode::getImage, this);

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

}   // namespace verilook_ros

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "face_recognition_verilook");

    ros::NodeHandle nh("~");

    verilook_ros::FaceRecognitionVerilookNode face_detection_verilook_node(nh);

    // Start ROS node. We need at least two threads so that VeriLook can be
    // supplied with images in the middle of a service call.
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
