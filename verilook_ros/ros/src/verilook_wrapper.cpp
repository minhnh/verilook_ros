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

/* Package */
#include <verilook_wrapper.h>

namespace verilook_ros
{

using Neurotec::Biometrics::NTemplateSize;
using Neurotec::Licensing::NLicense;
using Neurotec::NError;
using Neurotec::NErrorReport;

struct NeurotecObjects
{
    NeurotecObjects() :
        hSubject(NULL), hFace(NULL), /*biometricClient(NULL),*/ hBuffer(NULL), hLAttributes(NULL),
        hEnumerator(NULL), hBiometricTask(NULL), hImage(NULL), hBiometricStatus(NULL)
    { }

    ~NeurotecObjects()
    {
        NResult result = NObjectSet(NULL, (HNObject*) &hSubject);
        if (NFailed(result)) printErrorMsg("NeurotecObjects: clear hSubject failed", result);

        result = NObjectSet(NULL, (HNObject*) &hFace);
        if (NFailed(result)) printErrorMsg("NeurotecObjects: clear hFace failed", result);

//        result = NObjectSet(NULL, (HNObject*) &biometricClient);
//        if (NFailed(result)) printErrorMsg("NeurotecObjects: clear hBiometricClient failed", result);

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
    Neurotec::HNBuffer hBuffer;
    Neurotec::Biometrics::HNLAttributes hLAttributes;
    Neurotec::IO::HNFileEnumerator hEnumerator;
    Neurotec::Biometrics::HNBiometricTask hBiometricTask;
    HNImage hImage;
    Neurotec::HNString hBiometricStatus;
};

NResult enrollFaceFromImageFunction(std::string templateFileName, GetImageFunctionType getImage,
                                    FaceRecognitionVerilookNode* obj, NRect *pBoundingRect,
                                    NBiometricClient & biometricClient)
{
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
    result = Neurotec::Biometrics::NBiometricSetHasMoreSamples(neurotecObjects.hFace, NTrue);
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

    // set template size to large
    biometricClient.SetFacesTemplateSize(Neurotec::Biometrics::ntsLarge);

    //TODO: make this in the same scheme as other license components
    Neurotec::NBoolean hasEx = NFalse;
    hasEx = NLicense::IsComponentActivated("Biometrics.FaceSegmentsDetection");

    if (hasEx)
    {
        // set detect all facial features
        biometricClient.SetFacesDetectAllFeaturePoints(NTrue);
        biometricClient.SetFacesDetectBaseFeaturePoints(NTrue);
        biometricClient.SetFacesDetermineGender(NTrue);
    }

    return result;
}

void setupBiometricClient(NBiometricClient &biometricClient)
{
    try
    {
        const std::string dbPath = "/home/minh/.ros/data/verilook_ros/faces.db";
        biometricClient.SetDatabaseConnectionToSQLite(dbPath);
        biometricClient.SetCustomDataSchema(Neurotec::Biometrics::NBiographicDataSchema::Parse("(Thumbnail blob)"));

        if (!NLicense::IsComponentActivated("Biometrics.FaceSegmentation"))
        {
            biometricClient.SetFacesDetectAllFeaturePoints(false);
            biometricClient.SetFacesDetectBaseFeaturePoints(false);
            biometricClient.SetFacesDetermineGender(false);
            biometricClient.SetFacesDetermineAge(false);
            biometricClient.SetFacesRecognizeEmotion(false);
            biometricClient.SetFacesDetectProperties(false);
            biometricClient.SetFacesRecognizeExpression(false);
        }

        biometricClient.SetMatchingWithDetails(true);
        biometricClient.SetBiometricTypes(Neurotec::Biometrics::nbtFace);
        biometricClient.Initialize();

    }
    catch (NError & e)
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << ": setup biometric client failed: " << std::string(e.GetMessage()));
        throw e;
    }
}

void obtainVerilookLicenses()
{
    const std::string Components[] = LICENSE_COMPONENTS;
    NResult result = Neurotec::N_OK;
    ROS_INFO_STREAM(PACKAGE_NAME << ": obtaining licenses...");
    try
    {
        for (unsigned int i = 0; i < sizeof(Components)/sizeof(*Components); i++)
        {
            result = NLicense::ObtainComponents(LICENSE_SERVER, LICENSE_PORT, Components[i]);
            if (!result)
            {
                ROS_ERROR_STREAM(PACKAGE_NAME << ": license for " << Components[i] << " is not available");
            }
            else
            {
                ROS_INFO_STREAM(PACKAGE_NAME << ": license for " << Components[i] << " obtained successfully");
            }
        }
    }
    catch (NError& e)
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << ": obtaining licenses failed: " << std::string(e.GetMessage()));
        releaseVerilookLicenses();
        throw e;
    }
}

void releaseVerilookLicenses()
{
    const std::string Components[] = LICENSE_COMPONENTS;
    ROS_INFO_STREAM(PACKAGE_NAME << ": releasing licenses...");
    try
    {
        for (unsigned int i = 0; i < sizeof(Components)/sizeof(*Components); i++)
        {
            NLicense::ReleaseComponents(Components[i]);
        }
    }
    catch (Neurotec::NError& e)
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << ": releasing licenses failed: " << std::string(e.GetMessage()));
        throw e;
    }
}

NResult printErrorMsg(const std::string szErrorMessage, NResult result)
{
    ROS_ERROR_STREAM(PACKAGE_NAME << ": " << szErrorMessage);
    return NErrorReport(result);
}

NResult printErrorMsgWithLastError(const std::string szErrorMessage, NResult result)
{
    Neurotec::HNError hError = NULL;

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

NResult retrieveErrorCodeRecursive(NResult result, Neurotec::HNError hError)
{
    if (result == Neurotec::N_E_AGGREGATE && hError != NULL)
    {
        Neurotec::HNError hInnerError = NULL;
        Neurotec::NErrorGetInnerErrorEx(hError, &hInnerError);
        Neurotec::NErrorGetCodeEx(hInnerError, &result);
        result = retrieveErrorCodeRecursive(result, hInnerError);
        NObjectSet(NULL, (HNObject *) &hInnerError);
    }
    return result;
}

}   // namespace verilook_ros
