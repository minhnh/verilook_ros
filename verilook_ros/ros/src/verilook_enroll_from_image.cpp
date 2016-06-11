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

void VerilookEnrollFromImage::extractTemplate(std::string &fileName)
{
    setBiometricClientParams();
    subject_ = Neurotec::Biometrics::NSubject();
    Neurotec::Biometrics::NFace face;
    face.SetFileName(fileName);
    subject_.GetFaces().Add(face);
    Neurotec::NAsyncOperation operation = biometricClient_.CreateTemplateAsync(subject_);
    operation.AddCompletedCallback(&VerilookEnrollFromImage::onCreateTemplateCompletedCallback, this);
}

void VerilookEnrollFromImage::onCreateTemplateCompletedCallback()
{

}

void VerilookEnrollFromImage::setBiometricClientParams()
{

}

void VerilookEnrollFromImage::initializeBiometricParams()
{
    // Can run GetFacesMaximalRoll and GetFacesMaximalYaw from biometric client here
}

}


