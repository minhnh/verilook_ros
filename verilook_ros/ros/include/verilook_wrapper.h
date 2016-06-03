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
#include <NLicensing.hpp>

/* Definitions */
#define LICENSE_COMPONENTS \
{\
    "Biometrics.FaceDetection", \
    "Devices.Cameras", \
    "Biometrics.FaceExtraction", \
    "Biometrics.FaceMatching" \
}
#define LICENSE_SERVER  "/local"
#define LICENSE_PORT    "5000"

namespace verilook_ros
{

void obtainVerilookLicenses();
void releaseVerilookLicenses();
Neurotec::NResult printErrorMsgWithLastError(const std::string szErrorMessage, Neurotec::NResult result);
Neurotec::NResult retrieveErrorCodeRecursive(Neurotec::NResult result, Neurotec::HNError hError);

}   // namespace verilook_ros

#endif /* VERILOOK_ROS_VERILOOK_WRAPPER_H_ */
