/* Copyright 2016 Bonn-Rhein-Sieg University
 *
 * verilook_ros.h
 *
 *  Created on: May 27, 2016
 *      Author: minh
 */

#ifndef VERILOOK_ROS_VERILOOK_WRAPPER_H_
#define VERILOOK_ROS_VERILOOK_WRAPPER_H_

/* System */
#include <vector>

/* Neurotec */
#include <NCore.hpp>
#include <Core/NTypes.hpp>
#include <Core/NDefs.h>
#include <NLicensing.hpp>
#include <Images/NImage.hpp>
#include <Geometry/NGeometry.hpp>
#include <NBiometricClient.hpp>

/* Package */
#include "face_recognition_verilook_node.h"

namespace verilook_ros
{
/* Forward declarations */
class FaceRecognitionVerilookNode;

/* Definitions */
#define LICENSE_COMPONENTS \
{\
    "Biometrics.FaceDetection", \
    "Biometrics.FaceExtraction", \
    "Biometrics.FaceMatching" \
}
#define LICENSE_SERVER  "/local"
#define LICENSE_PORT    "5000"

using Neurotec::NCore;
using Neurotec::NResult;
using Neurotec::NFailed;
using Neurotec::NObjectSet;
using Neurotec::HNObject;
using Neurotec::NTrue;
using Neurotec::NFalse;

typedef void ( FaceRecognitionVerilookNode::* GetImageFunctionType )(Neurotec::Images::HNImage*);

class VerilookWrapper {
public:
    VerilookWrapper(Neurotec::Biometrics::Client::NBiometricClient & biometricClient);
    ~VerilookWrapper();
    void enroll(GetImageFunctionType getImage, FaceRecognitionVerilookNode * obj);
    void createTemplate(GetImageFunctionType getImage, FaceRecognitionVerilookNode * obj);
    void setSubjectID(std::string);
private:
    void onCreateTemplateCompleted(Neurotec::Biometrics::NBiometricTask createTempalteTask);
    void onEnrollCompleted(Neurotec::Biometrics::NBiometricTask enrollTask);
    void onIdentifyCompleted(Neurotec::Biometrics::NBiometricTask identifyTask);

    void onAsyncOperationStarted(Neurotec::NAsyncOperation operation);
    void onAsyncOperationCompleted(Neurotec::NAsyncOperation operation);

    static void asyncOperationCompletedCallback(Neurotec::EventArgs args);

    void setBiometricClientParams();
    void setupBiometricClient();

    Neurotec::Biometrics::Client::NBiometricClient m_biometricClient;
    Neurotec::Biometrics::NBiometricOperations m_currentOperations;
    std::vector<Neurotec::NAsyncOperation> m_asyncOperations;
    bool m_isSegmentationActivated;
    std::string m_subjectID;
};

void obtainVerilookLicenses();
void releaseVerilookLicenses();
NResult printErrorMsg(const std::string szErrorMessage, NResult result);
NResult printErrorMsgWithLastError(const std::string szErrorMessage, NResult result);
NResult retrieveErrorCodeRecursive(NResult result, Neurotec::HNError hError);

}   // namespace verilook_ros

#endif /* VERILOOK_ROS_VERILOOK_WRAPPER_H_ */
