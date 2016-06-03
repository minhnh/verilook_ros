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
using Neurotec::HNError;
using Neurotec::NErrorReport;

struct NeurotecObjects
{
    NeurotecObjects() :
        hSubject(NULL), hFace(NULL), hBiometricClient(NULL), hBuffer(NULL), hLAttributes(NULL),
        hEnumerator(NULL), hBiometricTask(NULL), hImage(NULL), hBiometricStatus(NULL)
    { }

    ~NeurotecObjects()
    {
        NResult result = NObjectSet(NULL, (HNObject*) &hSubject);
        if (NFailed(result)) printErrorMsg("NeurotecObjects: clear hSubject failed", result);

        result = NObjectSet(NULL, (HNObject*) &hFace);
        if (NFailed(result)) printErrorMsg("NeurotecObjects: clear hFace failed", result);

        result = NObjectSet(NULL, (HNObject*) &hBiometricClient);
        if (NFailed(result)) printErrorMsg("NeurotecObjects: clear hBiometricClient failed", result);

        result = NObjectSet(NULL, (HNObject*) &hBuffer);
        if (NFailed(result)) printErrorMsg("NeurotecObjects: clear hBuffer failed", result);

        result = NObjectSet(NULL, (HNObject*) &hLAttributes);
        if (NFailed(result)) printErrorMsg("NeurotecObjects: clear hLAttributes failed", result);

        result = NObjectSet(NULL, (HNObject*) &hEnumerator);
        if (NFailed(result)) printErrorMsg("NeurotecObjects: clear hEnumerator failed", result);

        result = NObjectSet(NULL, (HNObject*) &hBiometricTask);
        if (NFailed(result)) printErrorMsg("NeurotecObjects: clear hBiometricTask failed", result);

        result = NObjectSet(NULL, (HNObject*) &hImage);
        if (NFailed(result)) printErrorMsg("NeurotecObjects: clear hImage failed", result);

        result = Neurotec::NStringSet(NULL, &hBiometricStatus);
        if (NFailed(result)) printErrorMsg("NeurotecObjects: clear hBiometricStatus failed", result);
    }

    Neurotec::Biometrics::HNSubject hSubject;
    Neurotec::Biometrics::HNFace hFace;
    Neurotec::Biometrics::Client::HNBiometricClient hBiometricClient;
    Neurotec::HNBuffer hBuffer;
    Neurotec::Biometrics::HNLAttributes hLAttributes;
    Neurotec::IO::HNFileEnumerator hEnumerator;
    Neurotec::Biometrics::HNBiometricTask hBiometricTask;
    HNImage hImage;
    Neurotec::HNString hBiometricStatus;
};

NResult enrollFaceFromImageFunction(std::string templateFileName, GetImageType getImage,
                                    FaceDetectionVerilookNode* obj, NRect *pBoundingRect)
{
    using Neurotec::Biometrics::Client::NBiometricClientCreate;
    NResult result = Neurotec::N_OK;

    NeurotecObjects neurotecObjects;

    // create subject
    result = Neurotec::Biometrics::NSubjectCreate(&neurotecObjects.hSubject);
    if (NFailed(result))
    {
        result = printErrorMsgWithLastError(N_T("NSubjectCreate() failed (result = %d)!"), result);
        return result;
    }

    // create face for the subject
    result = Neurotec::Biometrics::NFaceCreate(&neurotecObjects.hFace);
    if (NFailed(result))
    {
        result = printErrorMsgWithLastError(N_T("NFaceCreate() failed (result = %d)!"), result);
        return result;
    }

    // set that face will be captured from image stream
    result = Neurotec::Biometrics::NBiometricSetHasMoreSamples(neurotecObjects.hFace, Neurotec::NTrue);
    if (NFailed(result))
    {
        result = printErrorMsgWithLastError(N_T("NBiometricSetHasMoreSamples() failed (result = %d)!"), result);
        return result;
    }

    // set the face for the subject
    result = NSubjectAddFace(neurotecObjects.hSubject, neurotecObjects.hFace, NULL);
    if (NFailed(result))
    {
        result = printErrorMsgWithLastError(N_T("NSubjectAddFace() failed (result = %d)!"), result);
        return result;
    }

    // create biometric client
    result = NBiometricClientCreate(&neurotecObjects.hBiometricClient);
    if (NFailed(result))
    {
        result = printErrorMsgWithLastError(N_T("NBiometricClientCreate() failed (result = %d)!"), result);
        return result;
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

NResult printErrorMsg(const std::string szErrorMessage, NResult result)
{
    ROS_ERROR_STREAM(PACKAGE_NAME << ": " << szErrorMessage);
    return NErrorReport(result);
}

NResult printErrorMsgWithLastError(const std::string szErrorMessage, NResult result)
{
    HNError hError = NULL;

    ROS_ERROR_STREAM(PACKAGE_NAME << ": " << szErrorMessage << result);

    Neurotec::NErrorGetLastEx(0, &hError);
    if (hError)
    {
        Neurotec::NErrorReportEx(result, hError);
        result = retrieveErrorCodeRecursive(result, hError);
        NObjectSet(NULL, (HNObject *) &hError);
    }
    else
    {
        NErrorReport(result);
    }

    return result;
}

NResult retrieveErrorCodeRecursive(NResult result, HNError hError)
{
    if (result == Neurotec::N_E_AGGREGATE && hError != NULL)
    {
        HNError hInnerError = NULL;
        Neurotec::NErrorGetInnerErrorEx(hError, &hInnerError);
        Neurotec::NErrorGetCodeEx(hInnerError, &result);
        result = retrieveErrorCodeRecursive(result, hInnerError);
        NObjectSet(NULL, (HNObject *) &hInnerError);
    }
    return result;
}

}   // namespace verilook_ros
