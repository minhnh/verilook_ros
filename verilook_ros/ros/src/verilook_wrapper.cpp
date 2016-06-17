/* Copyright 2016 Bonn-Rhein-Sieg University
 *
 * verilook_wrapper.cpp
 *
 *  Created on: May 27, 2016
 *      Author: minh
 */
/* System */
#include <string>

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
using Neurotec::Geometry::NRect;

using Neurotec::Biometrics::NBiometricTask;
using Neurotec::Biometrics::Client::NBiometricClient;
using Neurotec::Biometrics::NSubject;

using Neurotec::Biometrics::NBiometricOperations;
using Neurotec::Biometrics::nboCreateTemplate;
using Neurotec::Biometrics::nboEnroll;
using Neurotec::Biometrics::nboEnrollWithDuplicateCheck;
using Neurotec::Biometrics::nboIdentify;
using Neurotec::Biometrics::nboClear;
using Neurotec::Biometrics::nboNone;

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

void VerilookWrapper::enroll(GetImageFunctionType getImage, FaceRecognitionVerilookNode * obj)
{
    m_currentOperations = nboEnrollWithDuplicateCheck;
    createTemplate(getImage, obj);
}

void VerilookWrapper::createTemplate(GetImageFunctionType getImage, FaceRecognitionVerilookNode * obj)
{
    NSubject subject;
    //TODO: check type of HNImage himage, may need to be NImage
    setBiometricClientParams();
    Neurotec::Biometrics::NFace face;
    Neurotec::Biometrics::HNImage himage;

    (obj->getImage)(&himage);

    face.SetImage(Neurotec::Images::NImage(himage, true));

    subject.GetFaces().Add(face);

    subject.SetId("minh");

    subject.SetMultipleSubjects(true);

    NBiometricTask task = m_biometricClient.CreateTask(nboCreateTemplate, subject);

    onAsyncOperationStarted(m_biometricClient.PerformTaskAsync(task));
}

void VerilookWrapper::onCreateTemplateCompleted(NBiometricTask createTempalteTask)
{
    ROS_INFO_STREAM(PACKAGE_NAME << ": onCreateTemplateCompleted");

//    int facesCount;
//    Neurotec::Biometrics::NBiometricStatus status;
//    NSubject subject;
//    NBiometricTask subTask = m_biometricClient.CreateTask(m_currentOperations, NULL);
//
//    subject = createTempalteTask.GetSubjects().Get(0);
//    NSubject::RelatedSubjectCollection relatedSubjects = subject.GetRelatedSubjects();
//    facesCount = 1 + relatedSubjects.GetCount();
//
//    status = subject.GetStatus();
//    std::string id = subject.GetId();
//    if (status == nbsOk && (m_currentOperations == nboEnroll || m_currentOperations == nboEnrollWithDuplicateCheck))
//    {
//        NFace face = subject.GetFaces().Get(0);
//        NLAttributes attributes = face.GetObjects().Get(0);
//        NImage thumbnail = attributes.GetThumbnail();
//
//        if (id == wxEmptyString)
//        {
//            EnrollDlg dlg(this, wxEmptyString, thumbnail);
//            if (dlg.ShowModal() == wxID_OK)
//            {
//                id = dlg.GetUserId();
//                subject.SetId(id);
//            }
//            else
//            {
//                EnableControls();
//                StartCreateTemplateFromCamera();
//                return;
//            }
//        }
//    }
//
//    AppendTextLine(wxString::Format(wxT("detected %d face(s) in '%s':"), facesCount, id.c_str()));
//    for (int i = 0; i < facesCount; i++)
//    {
//        bool successful = false;
//        if (i > 0) subject = relatedSubjects.Get(i - 1);
//        status = subject.GetStatus();
//        statusString = "Liveness check failed";
//        if (status != nbsTimeout) statusString = NEnum::ToString(NBiometricTypes::NBiometricStatusNativeTypeOf(), status);
//        successful = status == nbsOk;
//        AppendText(wxString::Format(wxT(" > create template %s, status = %s\n"), (successful ? "successful" : "failed"), statusString.c_str()));
//        if (successful)
//        {
//            if (i > 0)
//            {
//                wxString relatedFaceId = wxString::Format(wxT("%s #%d"), id.c_str(), i + 1);
//                subject.SetId(relatedFaceId);
//            }
//
//            if (m_currentOperations == nboEnroll || m_currentOperations == nboEnrollWithDuplicateCheck)
//            {
//                NFace face = subject.GetFaces().Get(0);
//                NLAttributes attributes = face.GetObjects().Get(0);
//                NImage thumbnail = attributes.GetThumbnail();
//                if (thumbnail.GetHandle())
//                {
//                    NBuffer buffer = thumbnail.Save(NImageFormat::GetPng());
//                    subject.SetProperty(wxT("Thumbnail"), buffer);
//                }
//            }
//            subTask.GetSubjects().Add(subject);
//        }
//    }
//    if (subTask.GetSubjects().GetCount() > 0)
//        onAsyncOperationStarted(m_biometricClient.PerformTaskAsync(subTask));
//    else
//    {
//        // Create template from camera failed, restart capture
//        //createTemplate();
//        ROS_INFO_STREAM(PACKAGE_NAME << ": onCreateTemplateCompleted: no subject!");
//    }
}

void VerilookWrapper::onEnrollCompleted(NBiometricTask enrollTask)
{
    ROS_INFO_STREAM(PACKAGE_NAME << ": onEnrollCompleted");

}

void VerilookWrapper::onIdentifyCompleted(NBiometricTask identifyTask)
{

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
        m_biometricClient.SetDatabaseConnectionToSQLite(dbPath);
        m_biometricClient.SetCustomDataSchema(Neurotec::Biometrics::NBiographicDataSchema::Parse("(Thumbnail blob)"));

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
                                    FaceRecognitionVerilookNode * obj, NRect *pBoundingRect,
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
