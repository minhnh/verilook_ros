/* Copyright 2016 Bonn-Rhein-Sieg University
 * Author: Minh Nguyen
 */

/* System */
#include <opencv/cv.h>

/* ROS */
#include <cv_bridge/cv_bridge.h>

/* Robocup */
#include <mcr_perception_msgs/FaceList.h>

/* Package */
#include "face_recognition_verilook_node.h"

namespace verilook_ros
{

using Neurotec::Images::HNImage;
using Neurotec::Images::NImage;
using Neurotec::Geometry::NRect;
using Neurotec::Biometrics::NBiometricTypes;

FaceRecognitionVerilookNode::FaceRecognitionVerilookNode(ros::NodeHandle &nh)
: m_nodeHandle(nh), m_verilookWrapper(NULL), m_imageTransport(nh), m_imageBatchSize(1)
{
    bool enableDatabase;
    std::string databasePath;
    int imageBatchSize;

    m_nodeHandle.param<int>("image_batch_size", imageBatchSize, 1);
    m_imageBatchSize = imageBatchSize;

    m_nodeHandle.param<bool>("enable_database", enableDatabase, false);
    if (enableDatabase)
    {
        m_nodeHandle.param<std::string>("database_path", databasePath, "data/verilook_ros/faces.db");
        ROS_INFO_STREAM(PACKAGE_NAME << ": saving templates to database at " << databasePath);
    }

    try
    {
        NCore::OnStart();

        obtainVerilookLicenses();

        m_verilookWrapper = new VerilookWrapper(m_biometricClient, enableDatabase, databasePath);
    }
    catch (Neurotec::NError & e)
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << ": FaceDetectionVerilookNode initialization failed: " << std::string(e.GetMessage()));
    }

    // Start camera service
//    ros::ServiceServer service = m_nodeHandle.advertiseService(
//            "create_face_template", &FaceDetectionVerilookNode::createTemplateServiceCallback, this);

    m_pub_imageProcessed = m_imageTransport.advertise("processed_image", 1);
    m_pub_faceImage = m_imageTransport.advertise("face_image", 1);

    m_pub_faceList = m_nodeHandle.advertise<mcr_perception_msgs::FaceList>("face_list", 1);
    m_sub_eventIn = m_nodeHandle.subscribe("event_in", 1, &FaceRecognitionVerilookNode::eventInCallback, this);
    m_sub_subjectID = m_nodeHandle.subscribe("subject_id", 1, &FaceRecognitionVerilookNode::subjectIDCallback, this);

    m_pub_eventOut = m_nodeHandle.advertise<std_msgs::String>("event_out", 1);

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

    NImage newImg = NImage::FromData(NPF_RGB_8U, p_image->width, p_image->height, p_image->step,
            p_image->step,&p_image->data[0], p_image->height*p_image->step);
    m_images.push_back(newImg);
    mp_image = p_image;

    cond.notify_one();
}

void FaceRecognitionVerilookNode::getImage(std::vector<NImage> & phImage)
{
    boost::unique_lock<boost::mutex> lock(mtx);
    boost::system_time const timeout = boost::get_system_time() + boost::posix_time::seconds(1);
    if (cond.timed_wait(lock, timeout, boost::bind(&FaceRecognitionVerilookNode::condFulfilled, this)))
    {
        if (m_images.empty())
        {
            ROS_WARN_STREAM(PACKAGE_NAME << ": No image in image vector");
        }
        else
        {
            phImage.push_back(m_images.back());
        }
    }
    else
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << "Timed out while waiting on ROS image stream.");
    }
}

void FaceRecognitionVerilookNode::getMultipleImages(std::vector<NImage> & phImage)
{
    boost::unique_lock<boost::mutex> lock(mtx);
    boost::system_time const timeout = boost::get_system_time() + boost::posix_time::seconds(1);
    if (cond.timed_wait(lock, timeout, boost::bind(&FaceRecognitionVerilookNode::condFulfilled, this)))
    {
        if (m_images.empty())
        {
            ROS_WARN_STREAM(PACKAGE_NAME << ": No image in image vector");
        }
        else
        {
            for (std::vector<NImage>::iterator it = m_images.begin(); it != m_images.end(); it++)
            {
                phImage.push_back(*it);
            }
        }
    }
    else
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << ": Timed out while waiting on ROS image stream.");
    }
}

bool FaceRecognitionVerilookNode::condFulfilled()
{
//    return image_buffer != NULL;
    return m_images.size() >= m_imageBatchSize;
}

void FaceRecognitionVerilookNode::eventInCallback(const std_msgs::String::Ptr &msg)
{
    image_transport::ImageTransport it(m_nodeHandle);

    if (m_images.size() > 0)
        m_images.clear();

    if (msg->data == "e_enroll")
    {
        m_sub_imageRaw = it.subscribe(
                "/usb_cam/image_raw", 10, &FaceRecognitionVerilookNode::imageMessageCallback, this);

        m_verilookWrapper->enroll(&FaceRecognitionVerilookNode::getMultipleImages, this);

        // Shutdown the image stream to save CPU usage
        m_sub_imageRaw.shutdown();
    }
    else if (msg->data == "e_identify")
    {
        m_sub_imageRaw = it.subscribe(
                "/usb_cam/image_raw", 10, &FaceRecognitionVerilookNode::imageMessageCallback, this);
        m_verilookWrapper->identify(&FaceRecognitionVerilookNode::getImage, this);

        // Shutdown the image stream to save CPU usage
        m_sub_imageRaw.shutdown();
    }
    else if (msg->data == "e_createTemplate")
    {
        m_sub_imageRaw = it.subscribe(
                "/usb_cam/image_raw", 10, &FaceRecognitionVerilookNode::imageMessageCallback, this);
        m_verilookWrapper->createTemplate(&FaceRecognitionVerilookNode::getMultipleImages, this);

        // Shutdown the image stream to save CPU usage
        m_sub_imageRaw.shutdown();
    }
    else if (msg->data == "e_showTemplate")
    {
        showProcessedImage();
    }
}

void FaceRecognitionVerilookNode::showProcessedImage()
{
    std::vector<VerilookFace> faces = m_verilookWrapper->getCurrentFaces();
    if (faces.size() <= 0)
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << ": no face recorded");
        return;
    }

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

    mcr_perception_msgs::FaceList faceList;

    for(std::vector<VerilookFace>::iterator p_face = faces.begin(); p_face != faces.end(); ++p_face) {
        Neurotec::Geometry::NRect boundingRect = p_face->m_attributes.GetBoundingRect();
        ROS_DEBUG("%s: found face at (%d, %d), width = %d, height = %d",
                PACKAGE_NAME, boundingRect.X, boundingRect.Y,
                boundingRect.Width, boundingRect.Height);

        // Extract face image only
        int offsetWidth = (int) boundingRect.Width * 0.1;
        int offsetHeight = (int) boundingRect.Height * 0.15;
        cv::Mat cropped_image(cv_image->image, cv::Rect(
                boundingRect.X - offsetWidth, boundingRect.Y - offsetHeight * 2,
                boundingRect.Width + offsetWidth * 2, boundingRect.Height + offsetHeight * 3));
        cv_bridge::CvImage faceOnly;
        faceOnly.encoding = sensor_msgs::image_encodings::BGR8;
        faceOnly.image = cropped_image;

        mcr_perception_msgs::Face faceMsg;
        faceMsg.name = p_face->m_id;
        faceMsg.image = *faceOnly.toImageMsg();

        faceList.faces.push_back(faceMsg);

        // Drawing
        std::stringstream info;
        info << p_face->m_id;
//        info << ", gender: " << std::string(Neurotec::NEnum::ToString(
//                NBiometricTypes::NGenderNativeTypeOf(), p_face->m_attributes.GetGender()));
//        info << ", expression: " << std::string(Neurotec::NEnum::ToString(
//                NBiometricTypes::NLExpressionNativeTypeOf(), p_face->m_attributes.GetExpression()));

        cv::Point2f pointTopLeft(boundingRect.X, boundingRect.Y);
        cv::Point2f pointBottomRight(boundingRect.X + boundingRect.Width, boundingRect.Y + boundingRect.Height);

        cv::RNG rng(12345);
        cv::Scalar colour(rng.uniform(125, 255), rng.uniform(125, 255), rng.uniform(125, 255));
        cv::Scalar colourDark(rng.uniform(0, 125), rng.uniform(0, 125), rng.uniform(0, 125));

        cv::Point pointText(boundingRect.X, boundingRect.Y - 5);
        int baseLine = 0;
        cv::Size text = cv::getTextSize(info.str().c_str(), cv::FONT_HERSHEY_PLAIN, 5.0, 2, &baseLine);
        cv::rectangle(cv_image->image, pointText + cv::Point(0, baseLine - 10),
                pointText + cv::Point(text.width, - text.height - 20), CV_RGB(0, 0, 0),
                CV_FILLED);
        cv::putText(cv_image->image, info.str().c_str(), pointText,
                    cv::FONT_HERSHEY_PLAIN, 5.0, colour, 2, CV_AA);

        cv::rectangle(cv_image->image, pointTopLeft, pointBottomRight, colour, 2, 8, 0);
    }

    m_pub_imageProcessed.publish(cv_image->toImageMsg());
    m_pub_faceImage.publish(faceList.faces[0].image);
    m_pub_faceList.publish(faceList);
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
//    NObjectSet(NULL, (HNObject*) &image_buffer);

    // Subscribe to the image stream
    image_transport::ImageTransport it(m_nodeHandle);
    image_transport::Subscriber sub = it.subscribe(
            "/usb_cam/image_raw", 10, &FaceRecognitionVerilookNode::imageMessageCallback, this);

    // Invoke the main big "create template" or "enroll face" routine.
    NRect boundingRect;
    NResult result = Neurotec::N_OK;
//    m_verilookWrapper->enroll(&FaceRecognitionVerilookNode::getImage, this);

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
