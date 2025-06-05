//
// Created by dell on 2024/6/6.
//

#ifndef RKNN_HK_V1_LONGRUNTIMING_H
#define RKNN_HK_V1_LONGRUNTIMING_H
#include <iostream>
#include <unordered_map>
#include <mutex>
#include <list>
#include "SportCountInterface.h"
#include "CommonStruct.h"
#include "Face.h"

using namespace std;

namespace obs{

    typedef struct {
        int personId = 0;
        bool endStatus = false;
        int preCheckStatus = 0;
        int checkStatus = 0;
        int recog_face_rate_ = 0;
        bool isRecog = false;
    }obs_runner_t;

    class LongRunTiming  : public SportCountInterface{

    public:
        LongRunTiming(const int sportModel,const std::string config_path,const cv::Rect checkRect);

        virtual ~LongRunTiming();

        void Execute(const int cameraIndex,const cv::Mat &frame,const int location, const int64_t pts,std::vector<obs_person_t> &persons) override;

        void AddPoseForwardCallBack(function<void(const cv::Mat&,const int, std::vector<obs_body_t>&)> callback_object) override
        {
            poseForward_ = callback_object;
        }

        void AddSegForwardCallBack(function<void(const cv::Mat&,const int, std::vector<obs_mask_t>&)> callback_object) override
        {
            segForward_ = callback_object;
        }

        void AddRecognitionFaceCallBack(function<void(const cv::Mat&, const int,const int,const int,const int)> callback_object) override
        {
            recognitionFace_ = callback_object;
        }

        void AddPersonForwardCallBack(function<void(const cv::Mat&,const int, std::vector<obs_persons_box_t>&)> callback_object)override
        {
            personForward_ = callback_object;
        }

        void AddFootForwardCallBack(function<void(const cv::Mat&,const int, std::vector<obs_foot_t>&)> callback_object) override
        {
            footForward_ = callback_object;
        }

        void SetRunningStatus(int bStatus) override {
            std::lock_guard<std::mutex>lock(mtx_);
            bStatus_ = bStatus;
        }

        void setPersonRecogedFaceIsTrue(int personId) override{
            std::lock_guard<std::mutex>lock(mtx_);
            if (longRunnerMap_.find(personId) != longRunnerMap_.end()){
                longRunnerMap_[personId].isRecog = true;
            }
        }

    protected:

        void checkTouchLine(const std::vector<obs_mask_t> &segMask,const int64_t pts,obs_person_t &person);

        void checkJumpGun(const std::vector<obs_mask_t> &segMask,const int64_t pts,obs_person_t &person);

        void checkEnding(const obs_body_t &body,const cv::Mat &frame,const obs_joint_t &point,const int64_t pts,obs_person_t &person);

        void checkEnding(const obs_persons_box_t &personRect,const cv::Mat &frame,const obs_joint_t &point,const int64_t pts,obs_person_t &person);

        void getToeTipsPoint(const std::vector<cv::Point2f> &masks,const float start_min_height_y,obs_joint_t &toe_tips_point,obs_joint_t &heel_point);

    private:
        LongRunTiming(const LongRunTiming&);
        const LongRunTiming& operator = (const LongRunTiming&);

        function<void(const cv::Mat&, const int,std::vector<obs_body_t>&)> poseForward_ = nullptr;
        function<void(const cv::Mat&, const int,std::vector<obs_mask_t>&)> segForward_ = nullptr;
        function<void(const cv::Mat&, const int,const int,const int,const int)> recognitionFace_ = nullptr;
        function<void(const cv::Mat&,const int, std::vector<obs_persons_box_t>&)> personForward_ = nullptr;
        function<void(const cv::Mat&,const int, std::vector<obs_foot_t>&)> footForward_ = nullptr;

        std::string app_root_path_;
        std::mutex mtx_;
        int bStatus_ = 0;
        obs_line_t startLine_,cheatLine_,endLine_;
        std::vector<obs_joint_t> startRect;
        std::vector<obs_joint_t> endRect;
        std::unordered_map<int,obs_runner_t>longRunnerMap_;
        cv::Rect checkRect_;
//        std::vector<obs_joint_t> regFaceRect_;
        int no_touch_num_ = 0;
        bool bTouchLine_ = false;
        bool bJumpGun_ = false;

    };
}

#endif //RKNN_HK_V1_LONGRUNTIMING_H
