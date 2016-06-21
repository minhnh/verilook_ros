/* Copyright 2016 Bonn-Rhein-Sieg University
 *
 * verilook_wrapper.cpp
 *
 *  Created on: May 27, 2016
 *      Author: minh
 */
/* System */
#include <string>

#include <Collections/NCollections.hpp>

/* Package */
#include <verilook_wrapper.h>

namespace verilook_ros
{

using Neurotec::Biometrics::NTemplateSize;
using Neurotec::Licensing::NLicense;
using Neurotec::NError;
using Neurotec::NErrorReport;
using Neurotec::NObject;
using Neurotec::NAsyncOperation;
using Neurotec::Images::HNImage;
using Neurotec::Images::NImage;
using Neurotec::Geometry::NRect;

using Neurotec::Biometrics::NBiometricTask;
using Neurotec::Biometrics::Client::NBiometricClient;
using Neurotec::Biometrics::NSubject;
using Neurotec::Biometrics::NFace;
using Neurotec::Biometrics::NLAttributes;
using Neurotec::Biometrics::NBiometricStatus;
using Neurotec::Biometrics::NBiometricTypes;

using Neurotec::Biometrics::NBiometricOperations;
using Neurotec::Biometrics::nboCreateTemplate;
using Neurotec::Biometrics::nboEnroll;
using Neurotec::Biometrics::nboEnrollWithDuplicateCheck;
using Neurotec::Biometrics::nboIdentify;
using Neurotec::Biometrics::nboClear;
using Neurotec::Biometrics::nboNone;
using Neurotec::Biometrics::nbsOk;

VerilookWrapper::VerilookWrapper(NBiometricClient & biometricClient)
: m_biometricClient(biometricClient), m_currentOperations(nboNone)
{
    using Neurotec::Licensing::NLicense;
    m_isSegmentationActivated = NLicense::IsComponentActivated("Biometrics.FaceSegmentsDetection");
    setupBiometricClient();
}

VerilookWrapper::~VerilookWrapper()
{
    m_biometricClient.Cancel();
    for (std::vector<NAsyncOperation>::iterator it = m_asyncOperations.begin();
            it != m_asyncOperations.end(); it++)
    {
        it->Cancel();
    }
}

void VerilookWrapper::enroll(GetImageFunctionType getImage, FaceRecognitionVerilookNode * obj, std::string subjectID)
{
    m_currentOperations = nboEnrollWithDuplicateCheck;
    m_subjectID = subjectID;
    createTemplate(getImage, obj);
}

void VerilookWrapper::onEnrollCompleted(NBiometricTask enrollTask)
{
    ROS_INFO_STREAM(PACKAGE_NAME << ": onEnrollCompleted");
    bool successful = false;
    NBiometricTask::SubjectCollection subjects = enrollTask.GetSubjects();
    int count = subjects.GetCount();
    for (int i = 0; i < count; i++)
    {
        NSubject subject = subjects.Get(i);
        NBiometricStatus status = subject.GetStatus();
        std::string statusString = Neurotec::NEnum::ToString(
                NBiometricTypes::NBiometricStatusNativeTypeOf(), status);
        std::string id = subject.GetId();
        successful = (status == nbsOk);
        ROS_INFO("%s: enroll subject '%s' %s, status = %s", PACKAGE_NAME, id.c_str(),
                (successful ? "successful" : "failed"), statusString.c_str());
    }
}

void VerilookWrapper::createTemplate(GetImageFunctionType getImage, FaceRecognitionVerilookNode * obj)
{
    NSubject subject;
    //TODO: check type of HNImage himage, may need to be NImage
    setBiometricClientParams();
    NFace face;
    Neurotec::Biometrics::HNImage himage;

    (obj->getImage)(&himage);

    face.SetImage(NImage(himage, true));

    subject.GetFaces().Add(face);

    subject.SetId(m_subjectID);

    subject.SetMultipleSubjects(true);

    NBiometricTask task = m_biometricClient.CreateTask(nboCreateTemplate, subject);

    onAsyncOperationStarted(m_biometricClient.PerformTaskAsync(task));
}

void VerilookWrapper::onCreateTemplateCompleted(NBiometricTask createTempalteTask)
{
    ROS_INFO_STREAM(PACKAGE_NAME << ": onCreateTemplateCompleted");

    int facesCount;
    NBiometricStatus status;
    NSubject subject;
    std::string id, statusString;
    NBiometricTask subTask = m_biometricClient.CreateTask(m_currentOperations, NULL);

    subject = createTempalteTask.GetSubjects().Get(0);
    NSubject::RelatedSubjectCollection relatedSubjects = subject.GetRelatedSubjects();
    facesCount = 1 + relatedSubjects.GetCount();

    status = subject.GetStatus();
    id = subject.GetId();
    if (status == nbsOk && (m_currentOperations == nboEnroll || m_currentOperations == nboEnrollWithDuplicateCheck))
    {
        NFace face = subject.GetFaces().Get(0);
        NLAttributes attributes = face.GetObjects().Get(0);
        NImage thumbnail = attributes.GetThumbnail();

        //TODO: set custome id here
        id = "minh";
        subject.SetId(id);
    }

    ROS_INFO("%s: detected %d face(s) in '%s':", PACKAGE_NAME, facesCount, id.c_str());

    for (int i = 0; i < facesCount; i++)
    {
        bool successful = false;
        if (i > 0) subject = relatedSubjects.Get(i - 1);
        status = subject.GetStatus();
        statusString = "Liveness check failed";
        if (status != Neurotec::Biometrics::nbsTimeout)
            statusString = Neurotec::NEnum::ToString(
                    NBiometricTypes::NBiometricStatusNativeTypeOf(), status);
        successful = status == Neurotec::Biometrics::nbsOk;
        ROS_INFO("%s: create template %s, status = %s", PACKAGE_NAME,
                (successful ? "successful" : "failed"), statusString.c_str());
        if (successful)
        {
            if (i > 0)
            {
                char buff[100];
                snprintf(buff, sizeof(buff), "%s #%d", id.c_str(), i + 1);
                std::string relatedFaceId = buff;
                subject.SetId(relatedFaceId);
            }

            if (m_currentOperations == nboEnroll || m_currentOperations == nboEnrollWithDuplicateCheck)
            {
                NFace face = subject.GetFaces().Get(0);
                NLAttributes attributes = face.GetObjects().Get(0);
                NImage thumbnail = attributes.GetThumbnail();
                if (thumbnail.GetHandle())
                {
                    Neurotec::IO::NBuffer buffer = thumbnail.Save(
                            Neurotec::Images::NImageFormat::GetPng());
                    subject.SetProperty("Thumbnail", buffer);
                }
            }
            subTask.GetSubjects().Add(subject);
        }
    }

    if (subTask.GetSubjects().GetCount() > 0)
    {
        onAsyncOperationStarted(m_biometricClient.PerformTaskAsync(subTask));
    }
    else
    {
        ROS_WARN_STREAM(PACKAGE_NAME << ": onCreateTemplateCompleted: no subject!");
    }
}

void VerilookWrapper::onIdentifyCompleted(NBiometricTask identifyTask)
{
    ROS_INFO_STREAM(PACKAGE_NAME << ": onIdentifyCompleted");

}

struct ObjectCompare : public std::unary_function<NObject, bool>
{
    NObject target;
    explicit ObjectCompare(NAsyncOperation item) : target(item) { }
    bool operator() (NAsyncOperation arg) { return NObject::Equals(target, arg); }
};

void VerilookWrapper::onAsyncOperationStarted(NAsyncOperation operation)
{
    operation.AddCompletedCallback(&VerilookWrapper::asyncOperationCompletedCallback, this);
    m_asyncOperations.push_back(operation);
}

void VerilookWrapper::onAsyncOperationCompleted(NAsyncOperation operation)
{
    std::vector<NAsyncOperation>::iterator it = std::find_if(
            m_asyncOperations.begin(), m_asyncOperations.end(), ObjectCompare(operation));
    if (it != m_asyncOperations.end())
        m_asyncOperations.erase(it);
}

void VerilookWrapper::asyncOperationCompletedCallback(Neurotec::EventArgs args)
{
    ROS_INFO_STREAM(PACKAGE_NAME << ": in async operation callback");
    VerilookWrapper * p_verilookWrapper = static_cast<VerilookWrapper*>(args.GetParam());
    NAsyncOperation operation(static_cast<HNObject>(args.GetObject<NAsyncOperation>().RefHandle()), true);

    p_verilookWrapper->onAsyncOperationCompleted(operation);

    if (!operation.IsCanceled())
    {
        NError error = operation.GetError();
        if (error.GetHandle())
            ROS_ERROR_STREAM(PACKAGE_NAME << ": async operation error: " << std::string(error.ToString()));
        else
        {
            Neurotec::NValue result = operation.GetResult();
            NBiometricTask task = result.ToObject(NBiometricTask::NativeTypeOf()).GetHandle();
            error = task.GetError();
            if (error.GetHandle())
                ROS_ERROR_STREAM(PACKAGE_NAME << ": async task error: " << std::string(error.ToString()));
            else
            {
                NBiometricOperations operations = task.GetOperations();
                if (operations == nboCreateTemplate)
                {
                    p_verilookWrapper->onCreateTemplateCompleted(task);
                }
                else if (operations == nboEnroll || operations == nboEnrollWithDuplicateCheck)
                {
                    p_verilookWrapper->onEnrollCompleted(task);
                }
                else if (operations == nboIdentify)
                {
                    p_verilookWrapper->onIdentifyCompleted(task);
                }
                else if (operations == nboClear)
                {
                    ROS_INFO_STREAM(PACKAGE_NAME << ": database cleared");
                }
            }
        }
    }
}

void VerilookWrapper::setSubjectID(std::string subjectID)
{
    m_subjectID = subjectID;
}

void VerilookWrapper::setBiometricClientParams()
{
    // Can run GetFacesMaximalRoll and SetFacesMaximalYaw from biometric client here
    m_biometricClient.SetFacesDetectAllFeaturePoints(m_isSegmentationActivated);
    m_biometricClient.SetFacesDetectBaseFeaturePoints(m_isSegmentationActivated);
    m_biometricClient.SetFacesDetermineGender(m_isSegmentationActivated);
    m_biometricClient.SetFacesDetermineAge(m_isSegmentationActivated);
    m_biometricClient.SetFacesDetectProperties(m_isSegmentationActivated);
    m_biometricClient.SetFacesRecognizeEmotion(m_isSegmentationActivated);
    m_biometricClient.SetFacesRecognizeExpression(m_isSegmentationActivated);
}

void VerilookWrapper::setupBiometricClient()
{
    try
    {
        const std::string dbPath = "/home/minh/.ros/data/verilook_ros/faces.db";
        //m_biometricClient.SetDatabaseConnectionToSQLite(dbPath);
        //m_biometricClient.SetCustomDataSchema(Neurotec::Biometrics::NBiographicDataSchema::Parse("(Thumbnail blob)"));

        if (!NLicense::IsComponentActivated("Biometrics.FaceSegmentation"))
        {
            m_biometricClient.SetFacesDetectAllFeaturePoints(false);
            m_biometricClient.SetFacesDetectBaseFeaturePoints(false);
            m_biometricClient.SetFacesDetermineGender(false);
            m_biometricClient.SetFacesDetermineAge(false);
            m_biometricClient.SetFacesRecognizeEmotion(false);
            m_biometricClient.SetFacesDetectProperties(false);
            m_biometricClient.SetFacesRecognizeExpression(false);
        }

        m_biometricClient.SetMatchingWithDetails(true);
        m_biometricClient.SetBiometricTypes(Neurotec::Biometrics::nbtFace);
        m_biometricClient.Initialize();

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
