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

#define DEFAULT_CLIENT_PROPERTIES \
    "Faces.CreateThumbnailImage=True;"\
    "Faces.DetectAllFeaturePoints=True;"\
    "Faces.DetermineGender=True;"\
    "Faces.DetermineAge=True;"\
    "Faces.DetectProperties=True;"\
    "Faces.RecognizeExpression=True;"\
    "Faces.RecognizeEmotion=True;"

using Neurotec::NCore;
using Neurotec::NResult;
using Neurotec::NFailed;
using Neurotec::NObjectSet;
using Neurotec::HNObject;
using Neurotec::NTrue;
using Neurotec::NFalse;

//typedef void ( FaceRecognitionVerilookNode::* GetImageFunctionType )(Neurotec::Images::HNImage*);
typedef void ( FaceRecognitionVerilookNode::* GetImageFunctionType )(std::vector<Neurotec::Images::NImage> &);

struct VerilookFace
{
    VerilookFace(std::string id, Neurotec::Biometrics::NLAttributes attributes)
        : m_id(id), m_attributes(attributes) {}
    std::string m_id;
    Neurotec::Biometrics::NLAttributes m_attributes;
};

class VerilookWrapper
{
public:
    VerilookWrapper(Neurotec::Biometrics::Client::NBiometricClient & biometricClient,
            bool enableDatabase, std::string databasePath);
    ~VerilookWrapper();
    void enroll(GetImageFunctionType getImage, FaceRecognitionVerilookNode * obj);
    void identify(GetImageFunctionType getImage, FaceRecognitionVerilookNode * obj);
    void createTemplate(GetImageFunctionType getImage, FaceRecognitionVerilookNode * obj,
            Neurotec::Biometrics::NBiometricOperations nextOperation=Neurotec::Biometrics::nboNone);
    void setSubjectID(std::string);
    std::vector<VerilookFace> getCurrentFaces();
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

    std::vector<VerilookFace> m_currentFaces;
    std::vector<Neurotec::NAsyncOperation> m_asyncOperations;

    bool m_isSegmentationActivated;
    std::string m_subjectID;
    bool m_enableDatabase;
    std::string m_databasePath;
};

void obtainVerilookLicenses();
void releaseVerilookLicenses();
NResult printErrorMsg(const std::string szErrorMessage, NResult result);
NResult printErrorMsgWithLastError(const std::string szErrorMessage, NResult result);
NResult retrieveErrorCodeRecursive(NResult result, Neurotec::HNError hError);

}   // namespace verilook_ros

#endif /* VERILOOK_ROS_VERILOOK_WRAPPER_H_ */
