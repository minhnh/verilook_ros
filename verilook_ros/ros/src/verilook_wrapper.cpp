/* Copyright 2016 Bonn-Rhein-Sieg University
 *
 * verilook_wrapper.cpp
 *
 *  Created on: May 27, 2016
 *      Author: minh
 */
/* System */
#include <string>

/* ROS */
#include <ros/ros.h>

/* Neurotec */
#include <NBiometrics.hpp>
#include <NBiometricClient.hpp>

/* Package */
#include <verilook_wrapper.h>

namespace verilook_ros
{

using Neurotec::Licensing::NLicense;

NResult enrollFaceFromImageFunction(std::string templateFileName, GetImageType getImage,
                                    FaceDetectionVerilookNode* obj, NRect *pBoundingRect)
{
    using Neurotec::Biometrics::Client::NBiometricClientCreate;
    NResult result = Neurotec::N_OK;

    Neurotec::Biometrics::HNSubject hSubject = NULL;
    Neurotec::Biometrics::HNFace hFace = NULL;
    Neurotec::Biometrics::Client::HNBiometricClient hBiometricClient = NULL;

    // create subject
    result = Neurotec::Biometrics::NSubjectCreate(&hSubject);
    if (NFailed(result))
    {
        result = printErrorMsgWithLastError(
                N_T(PACKAGE_NAME + ": NSubjectCreate() failed (result = %d)!"), result);
        goto FINALLY;
    }

    // create face for the subject
    result = Neurotec::Biometrics::NFaceCreate(&hFace);
    if (NFailed(result))
    {
        result = printErrorMsgWithLastError(
                N_T(PACKAGE_NAME + ": NFaceCreate() failed (result = %d)!"), result);
        goto FINALLY;
    }

    // set that face will be captured from image stream
    result = Neurotec::Biometrics::NBiometricSetHasMoreSamples(hFace, Neurotec::NTrue);
    if (NFailed(result))
    {
        result = printErrorMsgWithLastError(
                N_T(PACKAGE_NAME + ": NBiometricSetHasMoreSamples() failed (result = %d)!"), result);
        goto FINALLY;
    }

    // set the face for the subject
    result = NSubjectAddFace(hSubject, hFace, NULL);
    if (NFailed(result))
    {
        result = printErrorMsgWithLastError(
                N_T(PACKAGE_NAME + ": NSubjectAddFace() failed (result = %d)!"), result);
        goto FINALLY;
    }

    // create biometric client
    result = NBiometricClientCreate(&hBiometricClient);
    if (NFailed(result))
    {
        result = printErrorMsgWithLastError(
                N_T(PACKAGE_NAME + ": NBiometricClientCreate() failed (result = %d)!"), result);
        goto FINALLY;
    }

    return result;
}

void obtainVerilookLicenses()
{
    const std::string Components[] = LICENSE_COMPONENTS;
    NResult result = Neurotec::N_OK;
    ROS_INFO_STREAM(PACKAGE_NAME << ": obtaining licenses...");
    try
    {
        NCore::OnStart();
        for (unsigned int i = 0; i < sizeof(Components)/sizeof(*Components); i++)
        {
            result = NLicense::ObtainComponents(LICENSE_SERVER, LICENSE_PORT, Components[i]);
            if (!result)
            {
                ROS_ERROR_STREAM(PACKAGE_NAME << ": License for " << Components[i] << " is not available");
            }
            else
            {
                ROS_DEBUG_STREAM(PACKAGE_NAME << ": License for " << Components[i] << " obtained successfully");
            }
        }
    }
    catch (Neurotec::NError& e)
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << ": " << std::string(e.ToString()));
        releaseVerilookLicenses();
    }
}

void releaseVerilookLicenses()
{
    const std::string Components[] = LICENSE_COMPONENTS;
    ROS_INFO_STREAM(PACKAGE_NAME << ": releasing licenses");
    try
    {
        for (unsigned int i = 0; i < sizeof(Components)/sizeof(*Components); i++)
        {
            NLicense::ReleaseComponents(Components[i]);
        }
    }
    catch (Neurotec::NError& e)
    {
        ROS_ERROR_STREAM(std::string(e.ToString()));
    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
}

NResult printErrorMsgWithLastError(const std::string szErrorMessage, Neurotec::NResult result)
{
    Neurotec::HNError hError = NULL;

    ROS_ERROR_STREAM(szErrorMessage << result);

    Neurotec::NErrorGetLastEx(0, &hError);
    if (hError)
    {
        Neurotec::NErrorReportEx(result, hError);
        result = retrieveErrorCodeRecursive(result, hError);
        Neurotec::NObjectSet(NULL, (Neurotec::HNObject *) &hError);
    }
    else
    {
        Neurotec::NErrorReport(result);
    }

    return result;
}

NResult retrieveErrorCodeRecursive(Neurotec::NResult result, Neurotec::HNError hError)
{
    if (result == Neurotec::N_E_AGGREGATE && hError != NULL)
    {
        Neurotec::HNError hInnerError = NULL;
        Neurotec::NErrorGetInnerErrorEx(hError, &hInnerError);
        Neurotec::NErrorGetCodeEx(hInnerError, &result);
        result = retrieveErrorCodeRecursive(result, hInnerError);
        Neurotec::NObjectSet(NULL, (Neurotec::HNObject *) &hInnerError);
    }
    return result;
}

}   // namespace verilook_ros
