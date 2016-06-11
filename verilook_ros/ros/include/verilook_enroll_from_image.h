/*
 * verilook_enroll_from_image.h
 *
 *  Created on: Jun 11, 2016
 *      Author: minh
 */

#ifndef VERILOOK_ENROLL_FROM_IMAGE_H_
#define VERILOOK_ENROLL_FROM_IMAGE_H_

/* System */
#include <string>
/* Neurotec */
#include <NBiometricClient.hpp>
/* Package */
#include "verilook_wrapper.h"

namespace verilook_ros
{

using Neurotec::Biometrics::Client::NBiometricClient;

class VerilookEnrollFromImage
{
public:
    VerilookEnrollFromImage(NBiometricClient &biometricClient);
    ~VerilookEnrollFromImage();

private:
    void extractTemplate(GetImageFunctionType getImage);
    void onCreateTemplateCompletedCallback();
    void setBiometricClientParams();
    void initializeBiometricParams();

    bool isSegmentationActivated_;
    Neurotec::Biometrics::NSubject subject_;
    NBiometricClient biometricClient_;
};

}

#endif /* VERILOOK_ENROLL_FROM_IMAGE_H_ */
