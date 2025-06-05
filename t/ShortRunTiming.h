//
// Created by dell on 2024/6/6.
//

#ifndef RKNN_HK_V1_SHORTRUNTIMING_H
#define RKNN_HK_V1_SHORTRUNTIMING_H

#include <iostream>
#include <unordered_map>
#include <mutex>
#include "SportCountInterface.h"
#include "RunTimingCount.h"

using namespace std;


namespace obs {

    class ShortRunTiming : public SportCountInterface {

    public:
        ShortRunTiming(const int sportModel,const std::string config_path,const int cameraIndex = 0);

        virtual ~ShortRunTiming();

        void Execute(const int cameraIndex,const cv::Mat &frame,const int location, const int64_t pts,std::vector<obs_person_t> &persons) override;

        void AddPoseForwardCallBack(function<void(const cv::Mat&,const int, std::vector<obs_body_t>&)> callback_object) override
        {
            poseForward_ = callback_object;
        }

        void AddSegForwardCallBack(function<void(const cv::Mat&,const int, std::vector<obs_mask_t>&)> callback_object) override
        {
            segForward_ = callback_object;
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

    protected:
        void getToeTipsPoint(const std::vector<cv::Point2f> &masks,const float start_min_height_y,obs_joint_t &toe_tips_point,obs_joint_t &heel_point);

    private:
        ShortRunTiming(const ShortRunTiming&);
        const ShortRunTiming& operator = (const ShortRunTiming&);

        function<void(const cv::Mat&, const int, std::vector<obs_body_t>&)> poseForward_ = nullptr;
        function<void(const cv::Mat&, const int, std::vector<obs_mask_t>&)> segForward_ = nullptr;
        function<void(const cv::Mat&,const int, std::vector<obs_persons_box_t>&)> personForward_ = nullptr;
        function<void(const cv::Mat&,const int, std::vector<obs_foot_t>&)> footForward_ = nullptr;

        std::string obs_config_path_;
        int sportModel_ = 0;
        std::mutex mtx_;
        int bStatus_ = 0;
        std::unordered_map<int,std::shared_ptr<RunTimingCount>>runTimingCountMap_;
        int midRunway = 0;
    };
}


#endif //RKNN_HK_V1_SHORTRUNTIMING_H
