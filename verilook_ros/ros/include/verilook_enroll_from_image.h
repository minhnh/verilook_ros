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
/* Forward declarations */
class FaceRecognitionVerilookNode;

typedef void ( FaceRecognitionVerilookNode::* GetImageFunctionType )(Neurotec::Images::HNImage*);

using Neurotec::Biometrics::Client::NBiometricClient;

class VerilookEnrollFromImage
{
public:
    VerilookEnrollFromImage(NBiometricClient &biometricClient);
    ~VerilookEnrollFromImage();
    void extractTemplate(GetImageFunctionType getImage, FaceRecognitionVerilookNode * obj);

private:
    static void onCreateTemplateCompletedCallback(Neurotec::EventArgs args);
    void setBiometricClientParams();
    void initializeBiometricParams();

    bool m_isSegmentationActivated;
    Neurotec::Biometrics::NSubject m_subject;
    NBiometricClient m_biometricClient;
};

}

#endif /* VERILOOK_ENROLL_FROM_IMAGE_H_ */
