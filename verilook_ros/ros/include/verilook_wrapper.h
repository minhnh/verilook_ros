/* Copyright 2016 Bonn-Rhein-Sieg University
 *
 * verilook_ros.h
 *
 *  Created on: May 27, 2016
 *      Author: minh
 */

#ifndef VERILOOK_ROS_VERILOOK_WRAPPER_H_
#define VERILOOK_ROS_VERILOOK_WRAPPER_H_

/* Neurotec */
#include <NCore.hpp>
#include <Core/NTypes.hpp>
#include <Core/NDefs.h>
#include <face_recognition_verilook_node.h>
#include <NLicensing.hpp>
#include <Images/NImage.hpp>
#include <Geometry/NGeometry.hpp>
#include <NBiometricClient.hpp>

/* Package */

namespace verilook_ros
{

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
using Neurotec::Images::HNImage;
using Neurotec::Geometry::NRect;
using Neurotec::NObjectSet;
using Neurotec::HNObject;
using Neurotec::NTrue;
using Neurotec::NFalse;
using Neurotec::Biometrics::Client::NBiometricClient;

typedef void ( FaceRecognitionVerilookNode::* GetImageFunctionType )(HNImage*);

NResult enrollFaceFromImageFunction(std::string templateFileName, GetImageFunctionType getImage,
                                    FaceRecognitionVerilookNode* obj, NRect *pBoundingRect,
                                    NBiometricClient & biometricClient);
void setupBiometricClient(NBiometricClient &biometricClient);
void obtainVerilookLicenses();
void releaseVerilookLicenses();
NResult printErrorMsg(const std::string szErrorMessage, NResult result);
NResult printErrorMsgWithLastError(const std::string szErrorMessage, NResult result);
NResult retrieveErrorCodeRecursive(NResult result, Neurotec::HNError hError);

}   // namespace verilook_ros

#endif /* VERILOOK_ROS_VERILOOK_WRAPPER_H_ */
