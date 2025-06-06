//
// Created by dell on 2024/3/19.
//

#ifndef IROAD_PHYSICAL_SPORTS_RKNNHANDLE_H
#define IROAD_PHYSICAL_SPORTS_RKNNHANDLE_H
#include <iostream>
#include <vector>
#include <unordered_map>
#include "rknncxx.h"
#include "Face.h"
#include "LocationPoint.h"
#include "SportCountInterface.h"
#include "Record.h"
#include "OBSThreadPool.h"

#define SAVE_IMAGE_FLAG 0
#define TEST_IMAGE_FLAG 0
#define LOCATION_SIZE 5

using namespace std;

namespace obs {

    class RKNNHandle {

    public:
        RKNNHandle();

        ~RKNNHandle();

        void ReInitJson();

        int Init(const int width, const int height,const char *config_path,const int is_use_hk,const char *appKey,
                  const char *packageName, const char *macAdress, const char *androidId,const char *serialNumber,const char *imei);

        bool Start(const int sportMode);

        void Destroy();

        bool Inference(const int cameraIndex,const uint8_t *frameData, std::vector<obs_person_t> &persons);

        bool InferenceNew(const int cameraIndex,const uint8_t *frameData);

        bool GetSportReady(const uint8_t *frameData,const int sportMode);

        bool RegisterFace(const uint8_t *frameData,const long imageSzie,const char *userId,const char *userName,const int bFrom = 0);

        bool UpdateFace(const uint8_t *frameData,const long imageSzie,const char *userId,const char *userName,const int bFrom = 0);

        bool DeleteFace(const char * userId);

        bool RecognitionFace(const uint8_t *frameData,const long imageSzie,std::vector<obs_user_t> &users,const int bFrom = 0);

        bool DoGetFaceFeature(const cv::Mat &frame,const bool bRectArea,std::vector<float> &features);

        void SetFaceRecogCallbackFunc(function<void(const int,const int,const int,const int,const char *,const char *)> callbackObj){
            faceRecogCallbackFuncObj_ = callbackObj;
        }

        void SetRopeRecogCallbackFunc(function<void(const std::vector<int> &)> callbackObj){
            ropeRecogCallbackFuncObj_ = callbackObj;
        }

        void SetSportResultCallbackFunc(function<void(const std::vector<obs_person_t>&)> callbackObj){
            sportResultCallbackFuncObj_ = callbackObj;
        }

        void SetSportReadyResultCallbackFunc(function<void(const std::vector<obs_location_t> &)> callbackObj){
            sportReadyResultCallbackFuncObj_ = callbackObj;
        }

        void SetRunningStatus(int bStatus);

        bool StartRecord(const int cameraIndex,const char *movieFile);

        bool StopRecord(const int cameraIndex);

        bool CutKeyFrame(const int cameraIndex,const char *movieFile,std::unordered_map<int64_t, std::string>&keyFrameMap);

    public:
        function<void(const std::vector<int> &)> ropeRecogCallbackFuncObj_ = nullptr;
        function<void(const int,const int,const int,const int,const char *,const char *)> faceRecogCallbackFuncObj_ = nullptr;
        function<void(const std::vector<obs_person_t> &)> sportResultCallbackFuncObj_ = nullptr;
        function<void(const std::vector<obs_location_t> &)> sportReadyResultCallbackFuncObj_ = nullptr;
        SportCountInterface *sportCount_ = NULL;
        SportCountInterface *sportCount_2 = NULL;
        Face faceObj;
        int is_use_hk_ = 0;
        std::mutex sportMutex_;
        std::mutex sportMutex_1_;
        std::mutex sportMutex_2_;
        std::mutex sportMutex_3_;

        void DoGetSportReady(const cv::Mat &frame,const int sportMode,const int locationIndex);

    protected:

        void PoseForwardOld(const cv::Mat &frame,const int location, std::vector<obs_body_t> &bodies);

        void PoseForward(const cv::Mat &frame,const int location, std::vector<obs_body_t> &bodies);

        void PoseForward2(const cv::Mat &frame,const int location, std::vector<obs_body_t> &bodies);

        void SegForward(const cv::Mat &frame,const int location, std::vector<obs_mask_t> &segMasks);

        void FootForward(const cv::Mat &frame,const int location, std::vector<obs_foot_t> &foots);

        void HandForward(const cv::Mat &frame, const int location, std::vector<obs_hand_t> &hands);

        void PersonForward(const cv::Mat &frame, const int location, std::vector<obs_persons_box_t> &personRects);

        void PersonForward2(const cv::Mat &frame, const int location, std::vector<obs_persons_box_t> &personRects);

        void DoRecognitionFaceCallbackFunc(const cv::Mat &frame,const int flag,const int location,const int x,const int y);

        void InitCommonJson(const std::string &jsonFile, std::unordered_map<int, std::shared_ptr<LocationPoint>>& locationMap);

        void doSportReady(const int sportMode,const cv::Mat &frame,const int locationIndex, const std::vector<obs_body_t>&out_bodies,
                          std::unordered_map<int, std::shared_ptr<LocationPoint>>&locationMap);

        void ResetStartReadyInfo();

        void LoadFaceConfig();

    private:

        std::unordered_map<int, std::shared_ptr<LocationPoint>> _SkipRope_map;
        std::unordered_map<int, std::shared_ptr<LocationPoint>> _SitUp_map;
        std::unordered_map<int, std::shared_ptr<LocationPoint>> _PullUp_map;
        std::unordered_map<int, std::shared_ptr<LocationPoint>> _longJump_map;
        std::unordered_map<int, std::shared_ptr<LocationPoint>> _openCloseJump_map;
        std::unordered_map<int, std::shared_ptr<LocationPoint>> _highLegLift_map;
        std::unordered_map<int, std::shared_ptr<LocationPoint>> _squatting_map;
        std::map<int, obs_location_rect_t> locationRectMap_;

        int sportMode_ = 0;
        std::string obs_config_path_;
        int src_img_width_ = 0;
        int src_img_height_ = 0;

        bool bInit = false;
        RKNNCXX rknnEngine;
        std::unordered_map<int, std::shared_ptr<Record>> recordMap_;
        OBSThreadPool threadPool_1;
        OBSThreadPool threadPool_2;
        OBSThreadPool threadPool_3;
        OBSThreadPool threadPool_4;

        int frame_count_0 = 0;
        int frame_count_1 = 0;
        int frame_count_2 = 0;
        int print_count_ = 0;
        std::vector<obs_joint_t> faceRect_;
        cv::Rect faceRect_temp;
    };

}
#endif //IROAD_PHYSICAL_SPORTS_RKNNHANDLE_H
