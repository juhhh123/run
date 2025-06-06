//
// Created by dell on 2024/3/19.
//

#include "HttpTool.h"
#include <fstream>
#include <sstream>
#include "RKNNHandle.h"
#include "Flags.h"
#include "Logger.h"
#include "json.h"
#include "CommonStruct.h"
#include "Squatting.h"
#include "ShortRunTiming.h"
#include "LongRunTiming.h"
#include "authorize.h"

#define CHECK_PERSON_TIME_OUT 1000
#define CHECK_PERSON_READY_TIME_OUT 500
#define SHOULDER_NOSE_DISTANCE_FACTOR 0.3


extern "C"{
#include "libavformat/avformat.h"
#include "libavutil/base64.h"
}

namespace obs {

    std::string rotateObfuscate(const std::string &input, int positions);

    std::string rotateDeobfuscate(const std::string &input, int positions);

    std::string replaceObfuscate(const std::string &input);

    std::string replaceDeobfuscate(const std::string &input);

    bool verify_authorization(const char *appKey,const char *packageName,
                              const char *macAdress, const char *androidId,const char *serialNumber,const char *imei,int &bReqFlag);

    std::string obs_linc_;

    HttpTool httpTool;

    //const std::string addr = "http://192.168.51.36:22001";
    const std::string addr = "http://119.145.33.218:7080";

    void ResetStartLocationMap(std::unordered_map<int, std::shared_ptr<LocationPoint>>& StartLocation)
    {
        for (auto iter = StartLocation.begin(); iter != StartLocation.end(); iter++) {
            iter->second->reset();
        }
    }

    void RegisterFaceHandle(const std::shared_ptr<cv::Mat>framePtr,const std::string userId,const std::string userName,void *arg){

        if (framePtr->empty()){
            return;
        }
        RKNNHandle* pRknnnHandle = (RKNNHandle*)arg;
        std::vector<float> features;
        pRknnnHandle->DoGetFaceFeature(*framePtr,false,features);
        if (features.empty()){
            return;
        }

        if (!pRknnnHandle->faceObj.Register(userId.c_str(),userName.c_str(),features)){
            LOGE("Register face failed userId:%s",userId.c_str());
        }else{
            LOGE("Register face userId:%s,userName:%s",userId.c_str(),userName.c_str());
        }
    }

    void UpdateFaceHandle(const std::shared_ptr<cv::Mat>framePtr,const std::string userId,const std::string userName,void *arg){

        if (framePtr->empty()){
            return;
        }
        RKNNHandle* pRknnnHandle = (RKNNHandle*)arg;
        std::vector<float> features;
        pRknnnHandle->DoGetFaceFeature(*framePtr,false,features);
        if (features.empty()){
            return;
        }

        if (!pRknnnHandle->faceObj.Update(userId.c_str(),userName.c_str(),features)){
            LOGE("Update face failed userId:%s",userId.c_str());
        }
    }

    void QueryFaceHandle(const std::shared_ptr<cv::Mat>framePtr,const int flag, const int location,const int x,const int y, void *arg){

//        uint64_t start = TimesUtil::GetLocalMillTimeStamp();
        if (framePtr->empty()){
            return;
        }

        RKNNHandle* pRknnnHandle = (RKNNHandle*)arg;
        std::vector<float> features;
        pRknnnHandle->DoGetFaceFeature(*framePtr,false,features);
        if (features.empty()){
            return;
        }

        std::vector<obs_user_t> users;
        if (pRknnnHandle->faceObj.RecognitionFace(features,users)){
            if (pRknnnHandle->faceRecogCallbackFuncObj_){
                const obs_user_t obsUser = users.front();
                pRknnnHandle->faceRecogCallbackFuncObj_(flag,location,x,y,obsUser.userId,obsUser.userName);
                if (flag == 1){
                    pRknnnHandle->sportCount_->setPersonRecogedFaceIsTrue(location);
                }
                LOGE("flag-->%d,location-->%d, uid-->%s,uName-->%s\n",flag,location,obsUser.userId,obsUser.userName);
            }
        }
//        LOGE("location-->%d,timeOut -->%ld\n",location,TimesUtil::GetLocalMillTimeStamp() - start);
    }

    void SportReadyResultHandle(const std::shared_ptr<cv::Mat>framePtr,const int locationIndex,const int sportMode, void *arg){

        RKNNHandle* pRknnnHandle = (RKNNHandle*)arg;

        pRknnnHandle->DoGetSportReady(*framePtr,sportMode,locationIndex);
    }

    void SportCountHandle1(const int cameraIndex,const std::shared_ptr<cv::Mat>framePtr,const int location, const int64_t pts, void *arg){

        if (framePtr->empty()){
            return;
        }

        RKNNHandle* pRknnnHandle = (RKNNHandle*)arg;
        std::vector<obs_person_t> persons;

        std::lock_guard<std::mutex> lock1(pRknnnHandle->sportMutex_1_);
        if (cameraIndex == 0 || cameraIndex == 1){
            if (pRknnnHandle->sportCount_ != NULL){
                pRknnnHandle->sportCount_->Execute(cameraIndex,*framePtr,location,pts,persons);
            }

        }else if(cameraIndex == 2){

            if (pRknnnHandle->sportCount_2 != NULL){
                pRknnnHandle->sportCount_2->Execute(cameraIndex,*framePtr,location,pts,persons);
            }
        }

        if (pRknnnHandle->sportResultCallbackFuncObj_ && !persons.empty()){
            pRknnnHandle->sportResultCallbackFuncObj_(persons);
        }
    }

    void SportCountHandle2(const int cameraIndex,const std::shared_ptr<cv::Mat>framePtr,const int location, const int64_t pts, void *arg){

        if (framePtr->empty()){
            return;
        }

        RKNNHandle* pRknnnHandle = (RKNNHandle*)arg;
        std::vector<obs_person_t> persons;

        std::lock_guard<std::mutex> lock2(pRknnnHandle->sportMutex_2_);
        if (cameraIndex == 0 || cameraIndex == 1){
            if (pRknnnHandle->sportCount_ != NULL){
                pRknnnHandle->sportCount_->Execute(cameraIndex,*framePtr,location,pts,persons);
            }
        }else if(cameraIndex == 2){
            if (pRknnnHandle->sportCount_2 != NULL){
                pRknnnHandle->sportCount_2->Execute(cameraIndex,*framePtr,location,pts,persons);
            }
        }

        if (pRknnnHandle->sportResultCallbackFuncObj_ && !persons.empty()){
            pRknnnHandle->sportResultCallbackFuncObj_(persons);
        }
    }

    void SportCountHandle3(const int cameraIndex,const std::shared_ptr<cv::Mat>framePtr,const int location, const int64_t pts, void *arg){

        if (framePtr->empty()){
            return;
        }

        RKNNHandle* pRknnnHandle = (RKNNHandle*)arg;
        std::vector<obs_person_t> persons;

        std::lock_guard<std::mutex> lock3(pRknnnHandle->sportMutex_3_);
        if (cameraIndex == 0 || cameraIndex == 1){
            if (pRknnnHandle->sportCount_ != NULL){
                pRknnnHandle->sportCount_->Execute(cameraIndex,*framePtr,location,pts,persons);
            }
        }else if(cameraIndex == 2){
            if (pRknnnHandle->sportCount_2 != NULL){
                pRknnnHandle->sportCount_2->Execute(cameraIndex,*framePtr,location,pts,persons);
            }
        }

        if (pRknnnHandle->sportResultCallbackFuncObj_ && !persons.empty()){
            pRknnnHandle->sportResultCallbackFuncObj_(persons);
        }
    }

    RKNNHandle::RKNNHandle(){
        threadPool_1.init(OBS_THREAD_NUM);
        threadPool_2.init(OBS_THREAD_NUM);
        threadPool_3.init(OBS_THREAD_NUM);
        threadPool_4.init(OBS_FACE_THREAD_NUM);
        threadPool_1.start();
        threadPool_2.start();
        threadPool_3.start();
        threadPool_4.start();
    }

    RKNNHandle::~RKNNHandle(){
        threadPool_1.waitForAllDone(200);
        threadPool_2.waitForAllDone(200);
        threadPool_3.waitForAllDone(200);
        threadPool_4.waitForAllDone(200);
        threadPool_1.stop();
        threadPool_2.stop();
        threadPool_3.stop();
        threadPool_4.stop();
    }

    int RKNNHandle::Init(const int width, const int height, const char *config_path,const int is_use_hk,const char *appKey,
                          const char *packageName, const char *macAdress, const char *androidId,const char *serialNumber,const char *imei){

        if(bInit){
            Destroy();
        }

        bInit = false;
        obs_config_path_ = std::string(config_path);
        src_img_width_ = width;
        src_img_height_ = height;
        is_use_hk_ = is_use_hk;
        std::string logPath = obs_config_path_ + "/logs/day.txt";
        Logger::getInstance()->setLogPath(logPath);

        obs_linc_ = obs_config_path_ + "/config/lic.bin";

        int bReqFlag = 0;
        if(!verify_authorization(appKey,packageName,macAdress,androidId,serialNumber,imei,bReqFlag)){
            LOGE("verify key -2");
            return -2;
        }

        if(!rknnEngine.InitRKNN(config_path,width,height)){
            LogError("InitRKNN failed");
            return -1;
        }

        if(!faceObj.Init(config_path)){
            LogError("faceObj init failed");
            return 0;
        }

        ReInitJson();
        LoadFaceConfig();

        bInit = true;

        return 1;
    }

    void RKNNHandle::ReInitJson(){
        InitCommonJson(obs_config_path_ + "/config/sit_up.json", _SitUp_map);
        InitCommonJson(obs_config_path_ + "/config/pull_up.json", _PullUp_map);
        InitCommonJson(obs_config_path_ + "/config/skip_rope.json", _SkipRope_map);
        InitCommonJson(obs_config_path_ + "/config/long_jump.json", _longJump_map);
        InitCommonJson(obs_config_path_ + "/config/open_close_jump.json", _openCloseJump_map);
        InitCommonJson(obs_config_path_ + "/config/high_leg_lift.json", _highLegLift_map);
        InitCommonJson(obs_config_path_ + "/config/squatting.json", _squatting_map);
        LOGI("reInit successed");
    }

    void RKNNHandle::Destroy(){

        if(!bInit){
            return;
        }

        if(sportCount_ != NULL){
            delete sportCount_;
            sportCount_ = NULL;
        }

        if (sportCount_2 != NULL) {
            delete sportCount_2;
            sportCount_2 = NULL;
        }

        rknnEngine.UnInitRKNN();
        faceObj.UnInit();

        bInit = false;
    }

    //new 课目对象
    bool RKNNHandle::Start(const int sportMode){

        std::lock_guard<std::mutex> lock(sportMutex_);
        std::lock_guard<std::mutex> lock1(sportMutex_1_);
        std::lock_guard<std::mutex> lock2(sportMutex_2_);
        std::lock_guard<std::mutex> lock3(sportMutex_3_);
        if(!bInit){
            return false;
        }

        threadPool_1.clearJob();
        threadPool_2.clearJob();
        threadPool_3.clearJob();
        threadPool_4.clearJob();

        if(sportMode_ != sportMode){
            rknnEngine.SetSportCheck(sportMode,locationRectMap_);
            for (auto& obj : locationRectMap_) {
                LOGE("locationRectMap location-->%d,rect0:[%d,%d,%d,%d]\n",obj.first,obj.second.checkRect0.x,obj.second.checkRect0.y,obj.second.checkRect0.width,obj.second.checkRect0.height);
                if (sportMode == SPORT_SHORT_RUN_TIMING_50 || sportMode == SPORT_SHORT_RUN_TIMING_100 ||
                    sportMode == SPORT_LONG_RUN_TIMING_800 || sportMode == SPORT_LONG_RUN_TIMING_1000 ||
                    sportMode == SPORT_SHORT_RUN_TIMING_8x50 || sportMode == SPORT_LONG_RUN_TIMING_3000 ||
                    sportMode == SPORT_SERPENTINE_RUNNING ||sportMode == SPORT_ORIENTEERING_RUNNING ||
                    sportMode == SPORT_BASKET_BALL_ROUND || sportMode == SPORT_FOOT_BALL_ROUND || sportMode == SPORT_SHORT_RUN_TIMING_60){

                    LOGE("locationRectMap location-->%d,rect1:[%d,%d,%d,%d]\n",obj.first,obj.second.checkRect1.x,obj.second.checkRect1.y,obj.second.checkRect1.width,obj.second.checkRect1.height);
                }
            }
            sportMode_ = sportMode;
        }

        if (sportCount_ != NULL) {
            delete sportCount_;
            sportCount_ = NULL;
        }

        if (sportCount_2 != NULL) {
            delete sportCount_2;
            sportCount_2 = NULL;
        }

        switch (sportMode) {

            case SPORT_SQUATTING:{
                sportCount_ = new Squatting(obs_config_path_ + "/config/squatting.json");
            }
                break;

            case SPORT_SHORT_RUN_TIMING_50:{
                sportCount_ = new ShortRunTiming(SPORT_SHORT_RUN_TIMING_50,obs_config_path_);
                sportCount_->AddSegForwardCallBack(std::bind(&RKNNHandle::SegForward, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
            }
                break;

            case SPORT_SHORT_RUN_TIMING_8x50:{
                sportCount_ = new ShortRunTiming(SPORT_SHORT_RUN_TIMING_8x50,obs_config_path_);
                sportCount_2 = new ShortRunTiming(SPORT_SHORT_RUN_TIMING_8x50,obs_config_path_,2);
                sportCount_->AddSegForwardCallBack(std::bind(&RKNNHandle::SegForward, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
                sportCount_2->AddSegForwardCallBack(std::bind(&RKNNHandle::SegForward, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
            }
                break;

            default:
                return false;
        }

        if(sportMode == SPORT_SHORT_RUN_TIMING_50 || sportMode == SPORT_SHORT_RUN_TIMING_100 ||
           sportMode == SPORT_LONG_RUN_TIMING_800 || sportMode == SPORT_LONG_RUN_TIMING_1000 ||
           sportMode == SPORT_SHORT_RUN_TIMING_8x50 || sportMode == SPORT_LONG_RUN_TIMING_3000 ||
           sportMode == SPORT_SERPENTINE_RUNNING ||sportMode == SPORT_ORIENTEERING_RUNNING ||
           sportMode == SPORT_BASKET_BALL_ROUND || sportMode == SPORT_FOOT_BALL_ROUND || sportMode == SPORT_SHORT_RUN_TIMING_60){

            sportCount_->AddPersonForwardCallBack(std::bind(&RKNNHandle::PersonForward, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
            sportCount_->AddFootForwardCallBack(std::bind(&RKNNHandle::FootForward, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
            if (sportMode_ == SPORT_SHORT_RUN_TIMING_8x50){
                sportCount_2->AddPersonForwardCallBack(std::bind(&RKNNHandle::PersonForward2, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
                sportCount_2->AddFootForwardCallBack(std::bind(&RKNNHandle::FootForward, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
            }
        }else{
            sportCount_->AddPoseForwardCallBack(std::bind(&RKNNHandle::PoseForward, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
        }

        LOGE("start model %d",sportMode);

        return true;
    }

    bool RKNNHandle::Inference(const int cameraIndex,const uint8_t *frameData, std::vector<obs_person_t> &persons){

        if(!bInit || frameData == nullptr || cameraIndex >2 || cameraIndex < 0 || sportMode_ != SPORT_LONG_JUMP){
            return false;
        }

        int64_t pts = 0;
        cv::Mat input_img;
        if (is_use_hk_ == 1){
            Flags::YV12ToBGR(frameData, src_img_width_, src_img_height_, input_img);
        }else{
            Flags::YUVJ420pToBGR(frameData, src_img_width_, src_img_height_, input_img);
        }

        std::lock_guard<std::mutex> lock(sportMutex_);

        if (recordMap_.find(cameraIndex) != recordMap_.end()){
            pts = recordMap_[cameraIndex]->PushFrame(input_img);
        }

        if (cameraIndex == 0){
            if (locationRectMap_.find(1) != locationRectMap_.end()){
                const obs_location_rect_t &location = locationRectMap_[1];
                cv::Rect recgRect(location.checkRect0);
                cv::Mat out_img = input_img(recgRect).clone();
                if (sportCount_ != NULL){
                    sportCount_->Execute(cameraIndex,out_img,location.location,pts,persons);
                }
            }
        }

        return true;
    }

    //课目运动 输入图片
    //cameraIndex 摄像头序号
    bool RKNNHandle::InferenceNew(const int cameraIndex,const uint8_t *frameData){

        if(!bInit || frameData == nullptr || cameraIndex >2 || cameraIndex < 0 || sportMode_ == SPORT_LONG_JUMP){
            return false;
        }

        int64_t pts = 0;
        cv::Mat input_img;
        if (is_use_hk_ == 1){
            Flags::YV12ToBGR(frameData, src_img_width_, src_img_height_, input_img);
        }else{
            Flags::YUVJ420pToBGR(frameData, src_img_width_, src_img_height_, input_img);
        }

        std::lock_guard<std::mutex> lock(sportMutex_);
        if (recordMap_.find(cameraIndex) != recordMap_.end()){
            pts = recordMap_[cameraIndex]->PushFrame(input_img);
        }

        if (cameraIndex == 0){

            if(sportMode_ == SPORT_PULL_UP){

                if (locationRectMap_.find(1) != locationRectMap_.end()){
                    const obs_location_rect_t &location1 = locationRectMap_[1];
                    cv::Mat CropFrame = input_img(location1.checkRect0);
                    std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(CropFrame);
                    threadPool_1.exec(SportCountHandle1, cameraIndex,imgPtr, location1.location, pts, this);
                }

            }else if(sportMode_ == SPORT_SHORT_RUN_TIMING_50 || sportMode_ == SPORT_SHORT_RUN_TIMING_100 ||
                     sportMode_ == SPORT_LONG_RUN_TIMING_800 || sportMode_ == SPORT_LONG_RUN_TIMING_1000 ||
                     sportMode_ == SPORT_SHORT_RUN_TIMING_8x50 || sportMode_ == SPORT_LONG_RUN_TIMING_3000 ||
                     sportMode_ == SPORT_SERPENTINE_RUNNING ||sportMode_ == SPORT_ORIENTEERING_RUNNING ||
                     sportMode_ == SPORT_BASKET_BALL_ROUND || sportMode_ == SPORT_FOOT_BALL_ROUND || sportMode_ == SPORT_SHORT_RUN_TIMING_60){

                if (locationRectMap_.find(1) != locationRectMap_.end()){
                    const obs_location_rect_t &location = locationRectMap_[1];
                    cv::Mat CropFrame = input_img(location.checkRect0);
                    std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(CropFrame);
                    threadPool_1.exec(SportCountHandle1,cameraIndex,imgPtr, location.location, pts, this);
                }

                if (locationRectMap_.find(2) != locationRectMap_.end()){
                    const obs_location_rect_t &location = locationRectMap_[2];
                    cv::Mat CropFrame = input_img(location.checkRect0);
                    std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(CropFrame);
                    threadPool_2.exec(SportCountHandle2,cameraIndex,imgPtr, location.location, pts, this);
                }

            }else{//跳绳 .....

                if (locationRectMap_.find(1) != locationRectMap_.end()){
                    const obs_location_rect_t &location1 = locationRectMap_[1];
                    cv::Mat CropFrame = input_img(location1.checkRect0);
                    std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(CropFrame);
                    threadPool_1.exec(SportCountHandle1,cameraIndex, imgPtr, location1.location, pts, this);
                }

                if (locationRectMap_.find(2) != locationRectMap_.end()){
                    const obs_location_rect_t &location2 = locationRectMap_[2];
                    cv::Mat CropFrame = input_img(location2.checkRect0);
                    std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(CropFrame);
                    threadPool_2.exec(SportCountHandle2,cameraIndex, imgPtr, location2.location, pts, this);
                }

                if (locationRectMap_.find(3) != locationRectMap_.end()){
                    const obs_location_rect_t &location3 = locationRectMap_[3];
                    cv::Mat CropFrame = input_img(location3.checkRect0);
                    std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(CropFrame);
                    threadPool_3.exec(SportCountHandle3, cameraIndex,imgPtr, location3.location, pts, this);
                }
            }

        }else if(cameraIndex == 1){

//            LOGE("cameraIndex == 1");
            //终点1
            if(sportMode_ == SPORT_SHORT_RUN_TIMING_50 || sportMode_ == SPORT_SHORT_RUN_TIMING_100 ||
               sportMode_ == SPORT_LONG_RUN_TIMING_800 || sportMode_ == SPORT_LONG_RUN_TIMING_1000 ||
               sportMode_ == SPORT_SHORT_RUN_TIMING_8x50 || sportMode_ == SPORT_LONG_RUN_TIMING_3000 ||
               sportMode_ == SPORT_SERPENTINE_RUNNING ||sportMode_ == SPORT_ORIENTEERING_RUNNING ||
               sportMode_ == SPORT_BASKET_BALL_ROUND || sportMode_ == SPORT_FOOT_BALL_ROUND || sportMode_ == SPORT_SHORT_RUN_TIMING_60){

                if (locationRectMap_.find(1) != locationRectMap_.end()){
                    const obs_location_rect_t &location1 = locationRectMap_[1];
                    cv::Mat CropFrame = input_img(location1.checkRect1);
                    std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(CropFrame);
                    threadPool_2.exec(SportCountHandle2,cameraIndex, imgPtr, location1.location, pts, this);
                }
            }

        }else if(cameraIndex == 2){
//            LOGE("cameraIndex == 2");
            //终点2
            if (locationRectMap_.find(1) != locationRectMap_.end()) {
                const obs_location_rect_t &location1 = locationRectMap_[1];
                cv::Mat CropFrame = input_img(location1.checkRect2);
                std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(CropFrame);
                threadPool_3.exec(SportCountHandle3, cameraIndex,imgPtr, location1.location, pts, this);
            }
        }

        if (++print_count_ >= 100){
            LOGE("threadPool_1->%d,threadPool_2->%d,threadPool_3->%d,threadPool_4->%d\n",
                 threadPool_1.getJobNum(),threadPool_2.getJobNum(),threadPool_3.getJobNum(),threadPool_4.getJobNum());
            print_count_ = 0;
        }

        return true;
    }

    bool RKNNHandle::RegisterFace(const uint8_t *frameData,const long imageSzie,const char *userId,const char *userName,const int bFrom){

        if(!bInit || frameData == NULL){
            return false;
        }

        cv::Mat frame;
        if (bFrom == 0){//file

            std::vector<uint8_t> data(frameData, frameData + imageSzie);
            frame = cv::imdecode(data, cv::IMREAD_COLOR);
        }else if (bFrom == 1){//usb camera

            Flags::NV21ToBGR(frameData,  src_img_width_, src_img_height_, frame);
        }else if(bFrom == 2){//web camera

            Flags::YV12ToBGR(frameData, src_img_width_, src_img_height_, frame);
        }else{
            return false;
        }

        if (frame.empty()){
            return false;
        }

        std::shared_ptr<cv::Mat> framePtr = std::make_shared<cv::Mat>(frame);
        threadPool_4.exec(RegisterFaceHandle,framePtr,std::string(userId),std::string(userName),this);

        return true;
    }

    bool RKNNHandle::UpdateFace(const uint8_t *frameData,const long imageSzie,const char *userId,const char *userName,const int bFrom){

        if(!bInit || frameData == NULL){
            return false;
        }

        cv::Mat frame;
        if (bFrom == 0){//file

            std::vector<unsigned char> data(frameData, frameData + imageSzie);
            frame = cv::imdecode(data, cv::IMREAD_COLOR);
        }else if (bFrom == 1){//usb camera

            Flags::NV21ToBGR(frameData,  src_img_width_, src_img_height_, frame);
        }else if(bFrom == 2){//web camera

            Flags::YV12ToBGR(frameData, src_img_width_, src_img_height_, frame);
        }else{
            return false;
        }

        if (frame.empty()){
            return false;
        }

        std::shared_ptr<cv::Mat> framePtr = std::make_shared<cv::Mat>(frame);
        threadPool_4.exec(UpdateFaceHandle,framePtr,std::string(userId),std::string(userName),this);

        return true;
    }

    bool RKNNHandle::DeleteFace(const char * userId){

        if(!bInit){
            return false;
        }

        if(!faceObj.Delete(userId)){
            return false;
        }

        return true;
    }

    bool RKNNHandle::RecognitionFace(const uint8_t *frameData,const long imageSzie,std::vector<obs_user_t> &users,const int bFrom){

        if(!bInit || frameData == NULL){
            return false;
        }

        cv::Mat frame;
        if (bFrom == 0){//file

            std::vector<uint8_t> data(frameData, frameData + imageSzie);
            frame = cv::imdecode(data, cv::IMREAD_COLOR);
        }else if (bFrom == 1){//usb camera

            Flags::NV21ToBGR(frameData,  src_img_width_, src_img_height_, frame);
        }else if(bFrom == 2){//web camera

            Flags::YV12ToBGR(frameData, src_img_width_, src_img_height_, frame);
        }else{
            return false;
        }

        cv::Mat inputImg = frame(faceRect_temp).clone();

        std::vector<float> features;
        DoGetFaceFeature(inputImg,true,features);

//        std::string fileImg = obs_config_path_ + "/images/" + std::to_string(++print_count_) + ".jpg";
//        cv::imwrite(fileImg.c_str(),inputImg);

        if (features.empty()){
            return false;
        }

        if(!faceObj.RecognitionFace(features,users)){
            return false;
        }

        return true;
    }

    bool RKNNHandle::DoGetFaceFeature(const cv::Mat &frame,const bool bRectArea,std::vector<float> &features){

        if(!bInit || frame.empty()){
            return false;
        }

        std::vector<obs_face_t> faceRes;
        rknnEngine.FaceDectForward(frame,faceRes);
        if (faceRes.empty()){
//            LOGE("faceRes.empty()");
            return false;
        }

        std::map<float, obs_face_t> faceMap_;
        for (int i = 0; i < faceRes.size(); ++i) {
            obs_face_t obj = faceRes.at(i);
            if (bRectArea){
//                const obs_joint_t obsJoint1 = obs_joint_t(obj.ponit[0].x,obj.ponit[0].y);
//                const obs_joint_t obsJoint2 =  obs_joint_t(obj.ponit[1].x,obj.ponit[1].y);
//                if (!Flags::isPointInPolygon(faceRect_,obsJoint1) ||
//                    !Flags::isPointInPolygon(faceRect_,obsJoint2)){
//                    continue;
//                }

                if (obj.box.area() > 15000.0f){
                    faceMap_[obj.box.area()] = obj;
                }
            }else{
                if (obj.box.area() > 200.0f){
                    faceMap_[obj.box.area()] = obj;
                }
            }
        }

        auto iter = faceMap_.rbegin();
        if (iter == faceMap_.rend()){
            return false;
        }

        obs_face_t obj = iter->second;
        cv::Mat landmark = (cv::Mat_<float>(5, 2) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        for (int j = 0; j < 5; ++j) {
            landmark.at<float>(j, 0) = static_cast<float>(obj.ponit[j].x);
            landmark.at<float>(j, 1) = static_cast<float>(obj.ponit[j].y);
        }

        cv::Mat outImg;
        if (!Flags::ImgWarpAffine(frame,outImg,cv::Size(FACE_RECT_SIZE,FACE_RECT_SIZE),landmark)){
            return false;
        }

        rknnEngine.FaceFeatureForward(outImg,features);

        return !features.empty();
    }

    void RKNNHandle::PoseForwardOld(const cv::Mat &frame,const int location, std::vector<obs_body_t> &bodies){
        if(!bInit || frame.empty()){
            return;
        }

        rknnEngine.PoseForwardOld(frame,bodies,location);
    }

    void RKNNHandle::PoseForward(const cv::Mat &frame,const int location, std::vector<obs_body_t> &bodies){
        if(!bInit || frame.empty()){
            return;
        }

        rknnEngine.PoseForward(frame,bodies,location);
    }

    void RKNNHandle::PoseForward2(const cv::Mat &frame,const int location, std::vector<obs_body_t> &bodies){
        if(!bInit || frame.empty()){
            return;
        }

        rknnEngine.PoseForward2(frame,bodies,location);
    }

    void RKNNHandle::SegForward(const cv::Mat &frame,const int location, std::vector<obs_mask_t> &segMasks){

        if(!bInit || frame.empty()){
            return;
        }

        if (!rknnEngine.SegForward(frame,segMasks,location)){
            LOGE("rknnEngine SegForward false");
        }
    }

    void RKNNHandle::FootForward(const cv::Mat &frame,const int location,std::vector<obs_foot_t> &foots){

        if(!bInit || frame.empty()){
            return;
        }
        rknnEngine.FootForward(frame,foots,location);
    }

    void RKNNHandle::HandForward(const cv::Mat &frame,const int location,std::vector<obs_hand_t> &hands){
        if(!bInit || frame.empty()){
            return;
        }
        rknnEngine.HandForward(frame,hands,location);
    }

    void RKNNHandle::PersonForward(const cv::Mat &frame, const int location, std::vector<obs_persons_box_t> &personRects){
        if(!bInit || frame.empty()){
            return;
        }

        rknnEngine.PersonForward(frame,personRects,location);
    }

    void RKNNHandle::PersonForward2(const cv::Mat &frame, const int location, std::vector<obs_persons_box_t> &personRects){
        if(!bInit || frame.empty()){
            return;
        }

        rknnEngine.PersonForward2(frame,personRects,location);
    }

    void RKNNHandle::DoRecognitionFaceCallbackFunc(const cv::Mat &frame,const int flag,const int location,const int x,const int y){

        std::shared_ptr<cv::Mat> framePtr = std::make_shared<cv::Mat>(frame);
        threadPool_4.exec(QueryFaceHandle,framePtr,flag,location,x,y,this);
    }

    void RKNNHandle::SetRunningStatus(int bStatus){

        if(sportMode_ == SPORT_SHORT_RUN_TIMING_50 ||
           sportMode_ == SPORT_SHORT_RUN_TIMING_100 ||
           sportMode_ == SPORT_LONG_RUN_TIMING_800 ||
           sportMode_ == SPORT_LONG_RUN_TIMING_1000 ||
           sportMode_ == SPORT_LONG_RUN_TIMING_3000 ||
           sportMode_ == SPORT_SERPENTINE_RUNNING ||
           sportMode_ == SPORT_ORIENTEERING_RUNNING ||
           sportMode_ == SPORT_SHORT_RUN_TIMING_8x50||
           sportMode_ == SPORT_BASKET_BALL_ROUND ||
           sportMode_ == SPORT_FOOT_BALL_ROUND ||
           sportMode_ == SPORT_SHORT_RUN_TIMING_60){

            std::lock_guard<std::mutex> lock(sportMutex_);
            if(sportCount_ != NULL){
                sportCount_->SetRunningStatus(bStatus);
            }

            if (sportMode_ == SPORT_SHORT_RUN_TIMING_8x50){
                if (sportCount_2 != NULL){
                    sportCount_2->SetRunningStatus(bStatus);
                }
            }
        }
    }

    bool RKNNHandle::StartRecord(const int cameraIndex,const char *movieFile){

        if (movieFile == NULL){
            return false;
        }

        if (recordMap_.find(cameraIndex) != recordMap_.end()){
            recordMap_.erase(cameraIndex);
        }

        std::shared_ptr<Record> camerObj(new Record());
        if (!camerObj->Init(src_img_width_,src_img_height_)){
            return false;
        }
        if (!camerObj->StartRecord(movieFile)){
            return false;
        }
        recordMap_[cameraIndex] = camerObj;

        return true;
    }

    bool RKNNHandle::StopRecord(const int cameraIndex){

        if (recordMap_.find(cameraIndex) == recordMap_.end()){
            return false;
        }

        return recordMap_[cameraIndex]->StopRecord();
    }

    bool RKNNHandle::CutKeyFrame(const int cameraIndex,const char *movieFile,std::unordered_map<int64_t, std::string>&keyFrameMap){

        if (movieFile == NULL || keyFrameMap.empty()){
            LogError("movieFile == NULL or keyFrameMap empty");
            return false;
        }

        if (recordMap_.find(cameraIndex) == recordMap_.end()){
            return false;
        }

        return recordMap_[cameraIndex]->CutKeyFrame(movieFile,keyFrameMap);
    }

    void RKNNHandle::InitCommonJson(const std::string &jsonFile, std::unordered_map<int, std::shared_ptr<LocationPoint>>&locationMap)
    {
        locationMap.clear();
        Json::Reader json_reader;
        Json::Value js;
        ifstream infile(jsonFile.c_str(), ios::binary);
        if (!infile.is_open()) {
            LOGE("Open {%s} failed!", jsonFile.c_str());
            return;
        }

        if (!json_reader.parse(infile, js)) {
            LOGE("parse {%s} failed!", jsonFile.c_str());
            infile.close();
            return;
        }
        infile.close();

        size_t nPos = jsonFile.find("long_jump");
        if (nPos != std::string::npos){
            Json::Value startPointList = js["StartPoint"];
            int index = js["index"].asInt();
            std::shared_ptr<LocationPoint> locationObj(new LocationPoint());
            for (int i = 0; i < startPointList.size(); ++i) {
                float x = startPointList[i]["x"].asFloat();
                float y = startPointList[i]["y"].asFloat();
                locationObj->localRect.push_back(obs_joint_t(x,y));
            }
            locationObj->location = index;
            locationMap[locationObj->location] = locationObj;
        }else{
            Json::Value locationList = js["LocationList"];
            for (int i = 0; i < locationList.size(); ++i) {
                int index = locationList[i]["index"].asInt();
                Json::Value rectPointList = locationList[i]["RectPoint"];
                std::shared_ptr<LocationPoint> locationObj(new LocationPoint());
                for (int j = 0; j < rectPointList.size(); ++j) {
                    float x = rectPointList[j]["x"].asFloat();
                    float y = rectPointList[j]["y"].asFloat();
                    locationObj->localRect.push_back(obs_joint_t(x,y));
                }
                locationObj->location = index;
                locationMap[index] = locationObj;
            }
        }
    }

    bool RKNNHandle::GetSportReady(const uint8_t *frameData,const int sportMode){

        std::lock_guard<std::mutex> lock(sportMutex_);
        if(!bInit || frameData == nullptr){
            return false;
        }

        if(sportMode_ != sportMode){
            usleep(300000);
            ResetStartReadyInfo();
            rknnEngine.SetSportCheck(sportMode,locationRectMap_);
            for (auto& obj : locationRectMap_) {
                LOGE("locationRectMap location-->%d,rect:[%d,%d,%d,%d]\n",obj.first,obj.second.checkRect0.x,obj.second.checkRect0.y,obj.second.checkRect0.width,obj.second.checkRect0.height);
            }
            sportMode_ = sportMode;
        }

        if (++frame_count_0 >= 2){

            cv::Mat input_img;
            if (is_use_hk_ == 1){
                Flags::YV12ToBGR(frameData, src_img_width_, src_img_height_, input_img);
            }else{
                Flags::YUVJ420pToBGR(frameData, src_img_width_, src_img_height_, input_img);
            }

            if (input_img.empty()){
                return -1;
            }

            if(sportMode == SPORT_PULL_UP || sportMode == SPORT_LONG_JUMP){

                if (locationRectMap_.find(1) != locationRectMap_.end()) {
                    const obs_location_rect_t &location1 = locationRectMap_[1];
                    cv::Mat CropFrame = input_img(location1.checkRect0);
                    std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(CropFrame);
                    threadPool_1.exec(SportReadyResultHandle, imgPtr,location1.location,sportMode,this);
                }

            }else{

                if (locationRectMap_.find(1) != locationRectMap_.end()){
                    const obs_location_rect_t &location1 = locationRectMap_[1];
                    cv::Mat CropFrame = input_img(location1.checkRect0);
                    std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(CropFrame);
                    threadPool_1.exec(SportReadyResultHandle, imgPtr,location1.location,sportMode,this);
                }

                if (locationRectMap_.find(2) != locationRectMap_.end()){
                    const obs_location_rect_t &location2 = locationRectMap_[2];
                    cv::Mat CropFrame = input_img(location2.checkRect0);
                    std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(CropFrame);
                    threadPool_2.exec(SportReadyResultHandle,imgPtr,location2.location,sportMode,this);
                }

                if (locationRectMap_.find(3) != locationRectMap_.end()){
                    const obs_location_rect_t &location3 = locationRectMap_[3];
                    cv::Mat CropFrame = input_img(location3.checkRect0);
                    std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(CropFrame);
                    threadPool_3.exec(SportReadyResultHandle,imgPtr,location3.location,sportMode,this);
                }
            }

            frame_count_0 = 0;
        }


        if (++print_count_ >= 100){
            LOGE("threadPool_1->%d,threadPool_2->%d,threadPool_3->%d,threadPool_4->%d\n",
                 threadPool_1.getJobNum(),threadPool_2.getJobNum(),threadPool_3.getJobNum(),threadPool_4.getJobNum());
            print_count_ = 0;
        }

        return true;
    }

    void RKNNHandle::DoGetSportReady(const cv::Mat &frame, const int sportMode,const int locationIndex){

        std::vector<obs_body_t> out_bodies;
        if(!rknnEngine.PoseForward(frame,out_bodies,locationIndex)){
            return;
        }

        switch(sportMode){

            case SPORT_SKIP_ROPE:
                doSportReady(SPORT_SKIP_ROPE, frame,locationIndex,out_bodies, _SkipRope_map);
                break;

            case SPORT_SIT_UP:
                doSportReady(SPORT_SIT_UP, frame,locationIndex,out_bodies, _SitUp_map);
                break;

            case SPORT_PULL_UP:
                doSportReady(SPORT_PULL_UP, frame,locationIndex,out_bodies, _PullUp_map);
                break;

            case SPORT_LONG_JUMP:
                doSportReady(SPORT_LONG_JUMP, frame,locationIndex,out_bodies, _longJump_map);
                break;

            case SPORT_OPEN_CLOSE_JUMP:
                doSportReady(SPORT_OPEN_CLOSE_JUMP, frame,locationIndex,out_bodies, _openCloseJump_map);
                break;

            case SPORT_HIGH_LEG_LIFT:
                doSportReady(SPORT_HIGH_LEG_LIFT, frame,locationIndex,out_bodies, _highLegLift_map);
                break;

            case SPORT_SQUATTING:
                doSportReady(SPORT_SQUATTING, frame,locationIndex,out_bodies, _squatting_map);
                break;

            default:break;
        }

    }

    void RKNNHandle::doSportReady(const int sportMode,
                                  const cv::Mat &frame,
                                  const int locationIndex,
                                  const std::vector<obs_body_t>& out_bodies,
                                  std::unordered_map<int, std::shared_ptr<LocationPoint>>&locationPointMap){

        if (locationPointMap.find(locationIndex) == locationPointMap.end()) return;

        std::vector<obs_location_t> locations;
        for (auto iter = locationPointMap.begin(); iter != locationPointMap.end();iter++) {

            std::shared_ptr<LocationPoint> obj = iter->second;
            if (locationIndex == 1){
                if (obj->location != 1) {
                    continue;
                }
            }else if (locationIndex == 3){
                if (obj->location != 3) {
                    continue;
                }
            }else if (locationIndex == 2){
                if (obj->location != 2 && obj->location != 4 && obj->location != 5) {
                    continue;
                }
            }

            obs_location_t location;
            location.location = obj->location;
            bool exist_person = false;
            for (int k = 0; k < out_bodies.size(); k++) {
                obs_body_t body = out_bodies[k];
                obs_joint_t left_ankle = body.joints[OBS_JOINT_ANKLE_LEFT];
                obs_joint_t right_ankle = body.joints[OBS_JOINT_ANKLE_RIGHT];
                obs_joint_t left_hip = body.joints[OBS_JOINT_HIP_LEFT];
                obs_joint_t right_hip = body.joints[OBS_JOINT_HIP_RIGHT];
                obs_joint_t left_shoulder = body.joints[OBS_JOINT_SHOULDER_LEFT];
                obs_joint_t right_shoulder = body.joints[OBS_JOINT_SHOULDER_RIGHT];
                obs_joint_t left_eye = body.joints[OBS_JOINT_EYE_LEFT];
                obs_joint_t right_eye = body.joints[OBS_JOINT_EYE_RIGHT];
                obs_joint_t left_wrist = body.joints[OBS_JOINT_WRIST_LEFT];
                obs_joint_t right_wrist = body.joints[OBS_JOINT_WRIST_RIGHT];
                obs_joint_t neck = body.joints[OBS_JOINT_NECK];

                if (body.rect.area() < BOX_MIN_AREA) continue;

                if (sportMode == SPORT_SIT_UP ||
                    sportMode == SPORT_SIT_AND_REACH ||
                    sportMode == SPORT_SIT_UP_NEW ||
                    sportMode == SPORT_PUSH_UP ||
                    sportMode == SPORT_DOUBLE_LEVER ||
                    sportMode == SPORT_SUPPORT_MOVEMENT){

                    if(!Flags::isPointInPolygon(obj->localRect, left_hip) &&
                       !Flags::isPointInPolygon(obj->localRect, right_hip))
                    {
                        continue;
                    }

                }else if(sportMode == SPORT_PULL_UP || sportMode == SPORT_OVERHANG){

                    if(!Flags::isPointInPolygon(obj->localRect,left_eye)||
                       !Flags::isPointInPolygon(obj->localRect,right_eye)||
                       !Flags::isPointInPolygon(obj->localRect,left_wrist) ||
                       !Flags::isPointInPolygon(obj->localRect,right_wrist)||
                       !Flags::isPointInPolygon(obj->localRect, left_hip)||
                       !Flags::isPointInPolygon(obj->localRect, right_hip)){
                        continue;
                    }

                }else{

                    //if(sportMode == SPORT_LONG_JUMP){
//                                obs_joint_t l_ankle;
//                                obs_joint_t r_ankle;
//                                Flags::UndistortPoints(left_ankle,l_ankle);
//                                Flags::UndistortPoints(right_ankle,r_ankle);
//                                left_ankle = l_ankle;
//                                right_ankle = r_ankle;
//                            }

                    if(!Flags::isPointInPolygon(obj->localRect, left_ankle) &&
                       !Flags::isPointInPolygon(obj->localRect, right_ankle))
                    {
                        continue;
                    }

                }

                exist_person = true;
                obj->out_range_count_ = 0;
                if(!obj->b_first){
                    obj->b_first = true;
                    obj->startTimer(CHECK_PERSON_READY_TIME_OUT);
                }

                if (++obj->recog_rate_ >= RECOGNITION_RATE){

                    if (Flags::is_point_valid(left_shoulder) && Flags::is_point_valid(right_shoulder) && Flags::is_point_valid(neck)){

                        int width = std::abs(left_shoulder.x - right_shoulder.x);
                        float rectY = body.rect.y;
                        if (rectY * DIFFEN_FACE_MIN_HEIGHT > locationRectMap_[obj->location].checkRect0.y){
                            rectY = rectY * DIFFEN_FACE_MIN_HEIGHT;
                        }else{
                            rectY = locationRectMap_[obj->location].checkRect0.y;
                        }

                        cv::Rect headRect;
                        headRect.x = neck.x - (width * DIFFEN_FACE_WIDTH / 2) - locationRectMap_[locationIndex].checkRect0.x;
                        headRect.y = rectY - locationRectMap_[locationIndex].checkRect0.y;
                        headRect.width = width * DIFFEN_FACE_WIDTH;
                        headRect.height = std::abs(neck.y - rectY);

                        if (headRect.area() > 200.0f){
                            if (headRect.x > 0 &&
                                headRect.y > 0 &&
                                headRect.x + headRect.width < frame.cols &&
                                headRect.y + headRect.height < frame.rows){
                                cv::Mat bodyImg = frame(headRect);
                                std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(bodyImg);
                                threadPool_4.exec(QueryFaceHandle,imgPtr,0,obj->location,neck.x,body.rect.y,this);
                            }
                        }

                        location.faceRect.x = std::min(left_shoulder.x,right_shoulder.x);
                        location.faceRect.y = rectY;
                        location.faceRect.width = headRect.width;
                        location.faceRect.height = headRect.height;
                    }

                    obj->recog_rate_ = 0;
                }

                break;
            }

            if(obj->b_first && obj->isTimeOut()){
                if (exist_person){
                    location.exist_person = 1;
                    obj->enter_in_ = true;
//                    LOGE("location:%d,enter_in",obj->location);
                }else{
                    if(obj->out_range_count_ ++ > READY_OUT_RANGE_COUNT){
                        location.exist_person = 0;
                        obj->enter_in_ = false;
                    }
                }
            }

            locations.push_back(location);
        }

        if (sportReadyResultCallbackFuncObj_ && !locations.empty()){
            sportReadyResultCallbackFuncObj_(locations);
        }

    }

    void RKNNHandle::ResetStartReadyInfo()
    {
        ResetStartLocationMap(_SkipRope_map);
        ResetStartLocationMap(_SitUp_map);
        ResetStartLocationMap(_PullUp_map);
        ResetStartLocationMap(_longJump_map);
        ResetStartLocationMap(_openCloseJump_map);
        ResetStartLocationMap(_highLegLift_map);
        ResetStartLocationMap(_squatting_map);
    }

    void RKNNHandle::LoadFaceConfig(){
        std::string face_rect_path = obs_config_path_ + "/config/face_rect.json";
        Json::Reader json_reader;
        Json::Value js;
        ifstream infile(face_rect_path.c_str(), ios::binary);
        if(!infile.is_open())
        {
            LogError("Open {%s} failed!",face_rect_path);
            return;
        }

        if(!json_reader.parse(infile, js)) {
            LogError("parse {%s} failed!",face_rect_path);
            infile.close();
            return;
        }
        infile.close();

        const int x1 = js["x1"].asInt();
        const int y1 = js["y1"].asInt();
        const int x2 = js["x2"].asInt();
        const int y2 = js["y2"].asInt();
        faceRect_temp.x = x1;
        faceRect_temp.y = y1;
        faceRect_temp.width = std::abs(x1 - x2);
        faceRect_temp.height = std::abs(y1 - y2);
        LOGE("face rect [%d,%d,%d,%d]",x1,y1,x2,y2)

        faceRect_.clear();
        faceRect_.push_back(obs_joint_t(x1,y1));
        faceRect_.push_back(obs_joint_t(x1,y2));
        faceRect_.push_back(obs_joint_t(x2,y2));
        faceRect_.push_back(obs_joint_t(x2,y1));

    }


    std::string rotateObfuscate(const std::string &input, int positions) {
        int len = input.size();
        positions = positions % len;
        return input.substr(len - positions) + input.substr(0, len - positions);
    }

    std::string rotateDeobfuscate(const std::string &input, int positions) {
        return rotateObfuscate(input, input.size() - (positions % input.size()));
    }

    std::string replaceObfuscate(const std::string &input) {
        std::unordered_map<char, char> replaceMap = {
                {'a', 'z'}, {'b', 'y'}, {'c', 'x'}, {'d', 'w'},{'e','v'},
                {'z', 'a'}, {'y', 'b'}, {'x', 'c'}, {'w', 'd'},{'v','e'},
                {'A', 'Z'}, {'B', 'Y'}, {'C', 'X'}, {'D', 'W'},{'E','V'},
                {'Z', 'A'}, {'Y', 'B'}, {'X', 'C'}, {'W', 'D'},{'V','E'}
        };

        std::string output;
        for (char ch : input) {
            if (replaceMap.find(ch) != replaceMap.end()) {
                output += replaceMap[ch];
            } else {
                output += ch;
            }
        }
        return output;
    }

    std::string replaceDeobfuscate(const std::string &input) {

        std::unordered_map<char, char> reverseReplaceMap = {
                {'a', 'z'}, {'b', 'y'}, {'c', 'x'}, {'d', 'w'},{'e','v'},
                {'z', 'a'}, {'y', 'b'}, {'x', 'c'}, {'w', 'd'},{'v','e'},
                {'A', 'Z'}, {'B', 'Y'}, {'C', 'X'}, {'D', 'W'},{'E','V'},
                {'Z', 'A'}, {'Y', 'B'}, {'X', 'C'}, {'W', 'D'},{'V','E'}
        };

        std::string output;
        for (char ch : input) {
            if (reverseReplaceMap.find(ch) != reverseReplaceMap.end()) {
                output += reverseReplaceMap[ch];
            } else {
                output += ch;
            }
        }
        return output;
    }

    bool verify_authorization(const char *appKey,const char *packageName,
                              const char *macAdress, const char *androidId,const char *serialNumber,const char *imei,int &bReqFlag){


        if(appKey == NULL){
            LOGE("appKey == NULL\n");
            return false;
        }

        if(packageName == NULL ){
            LOGE("packageName == NULL\n");
            return false;
        }

        if(macAdress == NULL){
            LOGE("macAdress == NULL\n");
            return false;
        }

        if(androidId == NULL){
            LOGE("androidId == NULL\n");
            return false;
        }

        if(serialNumber == NULL){
            LOGE("serialNumber == NULL\n");
            return false;
        }

        bool bReq = false;
        std::string responseTxt;
        if(Flags::fileExists(obs_linc_)){
            LOGE("authorize->file");
            FILE *fp = NULL;
            fp = fopen(obs_linc_.c_str(), "rb");
            if (NULL == fp)
            {
                LOGE("fopen fail %s\n",strerror(errno));
                return false;
            }

            fseek(fp, 0, SEEK_END);
            long fileSize = ftell(fp);
            if(fileSize <= 0){
                fclose(fp);
                return false;
            }
            rewind(fp);

            unsigned char *data = (unsigned char*)malloc(fileSize);
            if(fread(data, 1, fileSize, fp) != fileSize){
                free(data);
                fclose(fp);
                LOGE("fread fail %s\n",strerror(errno));
                return false;
            }

            const std::string rotate_str = rotateDeobfuscate(std::string((char*)data,fileSize),fileSize/2);

            responseTxt = replaceDeobfuscate(rotate_str);
            fclose(fp);
            bReqFlag = 0;

            //LOGE("responseTxt->%s\n",responseTxt.c_str());
            //LOGE("responseTxt size->%d\n",responseTxt.length());

        }else{
            LOGE("authorize->file not exit");
            Json::FastWriter writer;
            Json::Value js;
            js["accreditCode"] = std::string(appKey);
            js["packageName"] = std::string(packageName);
            js["mac"] = std::string(macAdress);
            js["androidId"] = std::string(androidId);
            js["serialNumber"] = std::string(serialNumber);
            js["imei"] = "";

            std::string postData = writer.write(js);
            postData.pop_back();

            if(!httpTool.post(addr + "/accredit/activate",postData,responseTxt)){
                LOGE("authorize->file not exit and post error");
                return false;
            }
            bReq = true;
            bReqFlag = 1;
//            LOGE("src responseTxt->%s,size->%d\n",responseTxt.c_str(),responseTxt.length());
            //LOGE("src responseTxt size->%d\n",responseTxt.length());
        }

        if (!authorize(packageName, macAdress, androidId,serialNumber,imei,responseTxt)) {
            bReq = false;
            LOGE("authorize->false");
            return false;
        }

        if(bReq){
            const std::string replace_str = replaceObfuscate(responseTxt);
            const std::string rotate_str = rotateObfuscate(replace_str,replace_str.length()/2);
            //LOGE("rotate_str->%s,size->%d\n",rotate_str.c_str(),rotate_str.length());
            //LOGE("rotate_str size->%d\n",rotate_str.length());
            FILE *fp = NULL;
            fp = fopen(obs_linc_.c_str(), "wb");
            LOGE("addr:%s",obs_linc_.c_str());
            if (NULL == fp)
            {
                LOGE("fopen fail %s\n",strerror(errno));
                return false;
            }

            if(fwrite((unsigned char*)rotate_str.c_str(),1,rotate_str.length(),fp) != rotate_str.length()){
                fclose(fp);
                LOGE("fwrite fail %s\n",strerror(errno));
                return false;
            }
            fclose(fp);
        }
        return true;
    }
}
