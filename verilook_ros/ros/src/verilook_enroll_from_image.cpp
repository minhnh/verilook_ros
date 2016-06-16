/*
 * verilook_enroll_from_image.cpp
 *
 *  Created on: Jun 11, 2016
 *      Author: minh
 */

/* Neurotec */
#include <NLicensing.hpp>

/* Package */
#include "verilook_enroll_from_image.h"

namespace verilook_ros
{

VerilookEnrollFromImage::VerilookEnrollFromImage(NBiometricClient &biometricClient)
: m_biometricClient(biometricClient), m_subject(NULL)
{
    using Neurotec::Licensing::NLicense;
    m_isSegmentationActivated = NLicense::IsComponentActivated("Biometrics.FaceSegmentsDetection");
    initializeBiometricParams();
}

VerilookEnrollFromImage::~VerilookEnrollFromImage()
{

}

void VerilookEnrollFromImage::extractTemplate(GetImageFunctionType getImage, FaceRecognitionVerilookNode * obj)
{
    //TODO: check type of HNImage himage, may need to be NImage
    setBiometricClientParams();
    m_subject = Neurotec::Biometrics::NSubject();
    Neurotec::Biometrics::NFace face;
    Neurotec::Biometrics::HNImage himage;

    (obj->getImage)(&himage);

    face.SetImage(Neurotec::Images::NImage(himage, true));

    m_subject.GetFaces().Add(face);

    //TODO: set subject ID

    m_subject.SetMultipleSubjects(true);

    Neurotec::NAsyncOperation operation = m_biometricClient.CreateTemplateAsync(m_subject);
    operation.AddCompletedCallback(&VerilookEnrollFromImage::onCreateTemplateCompletedCallback, this);
}

void VerilookEnrollFromImage::onCreateTemplateCompletedCallback(Neurotec::EventArgs args)
{
    ROS_INFO_STREAM(PACKAGE_NAME << ": in create template callback");
    try
    {
        //TODO: Write to database or to file?
        //Neurotec::IO::NFile::WriteAllBytes(saveFileDialog.GetPath(), m_subject.GetTemplateBuffer());
    }
    catch (Neurotec::NError& e)
    {
        //wxExceptionDlg::Show(e);
    }
}

void VerilookEnrollFromImage::setBiometricClientParams()
{
    // Can run GetFacesMaximalRoll and SetFacesMaximalYaw from biometric client here
    m_biometricClient.SetFacesDetectAllFeaturePoints(m_isSegmentationActivated);
    m_biometricClient.SetFacesDetectBaseFeaturePoints(m_isSegmentationActivated);
    m_biometricClient.SetFacesDetermineGender(m_isSegmentationActivated);
    m_biometricClient.SetFacesDetermineAge(m_isSegmentationActivated);
    m_biometricClient.SetFacesDetectProperties(m_isSegmentationActivated);
    m_biometricClient.SetFacesRecognizeEmotion(m_isSegmentationActivated);
    m_biometricClient.SetFacesRecognizeExpression(m_isSegmentationActivated);
}

void VerilookEnrollFromImage::initializeBiometricParams()
{
    // Can run GetFacesMaximalRoll and GetFacesMaximalYaw from biometric client here
}

}


