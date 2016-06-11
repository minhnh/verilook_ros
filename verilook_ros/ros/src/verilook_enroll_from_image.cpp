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
: biometricClient_(biometricClient), subject_(NULL)
{
    using Neurotec::Licensing::NLicense;
    isSegmentationActivated_ = NLicense::IsComponentActivated("Biometrics.FaceSegmentsDetection");
    initializeBiometricParams();
}

VerilookEnrollFromImage::~VerilookEnrollFromImage()
{

}

void VerilookEnrollFromImage::extractTemplate(GetImageFunctionType getImage)
{
    //TODO: check type of HNImage himage, may need to be NImage
    setBiometricClientParams();
    subject_ = Neurotec::Biometrics::NSubject();
    Neurotec::Biometrics::NFace face;
    Neurotec::Biometrics::HNImage himage;
    getImage(&himage);
    face.SetImage(himage);
    subject_.GetFaces().Add(face);
    Neurotec::NAsyncOperation operation = biometricClient_.CreateTemplateAsync(subject_);
    operation.AddCompletedCallback(&VerilookEnrollFromImage::onCreateTemplateCompletedCallback, this);
}

void VerilookEnrollFromImage::onCreateTemplateCompletedCallback()
{

}

void VerilookEnrollFromImage::setBiometricClientParams()
{
    // Can run GetFacesMaximalRoll and SetFacesMaximalYaw from biometric client here
    biometricClient_.SetFacesDetectAllFeaturePoints(isSegmentationActivated_);
    biometricClient_.SetFacesDetectBaseFeaturePoints(isSegmentationActivated_);
    biometricClient_.SetFacesDetermineGender(isSegmentationActivated_);
    biometricClient_.SetFacesDetermineAge(isSegmentationActivated_);
    biometricClient_.SetFacesDetectProperties(isSegmentationActivated_);
    biometricClient_.SetFacesRecognizeEmotion(isSegmentationActivated_);
    biometricClient_.SetFacesRecognizeExpression(isSegmentationActivated_);
}

void VerilookEnrollFromImage::initializeBiometricParams()
{
    // Can run GetFacesMaximalRoll and GetFacesMaximalYaw from biometric client here
}

}


