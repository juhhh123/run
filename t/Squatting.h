//
// Created by Lige on 2024/4/27.
//

#ifndef RKNN_YOLOV5_ANDROID_APK_DEMO_SQUATTING_H
#define RKNN_YOLOV5_ANDROID_APK_DEMO_SQUATTING_H

#include "CommonStruct.h"
#include "SportCountInterface.h"
#include "SquattingCount.h"
#include <unordered_map>


namespace obs {
    class Squatting : public SportCountInterface {

    public:
        Squatting(const std::string config_path);
        virtual ~Squatting();

        void Execute(const int cameraIndex,const cv::Mat &frame,const int location,const int64_t pts,std::vector<obs_person_t> &persons) override;

        void AddPoseForwardCallBack(function<void(const cv::Mat&, const int, std::vector<obs_body_t>&)> callback_object) override
        {
            poseForward_ = callback_object;
        }


    private:
        Squatting(const Squatting&);
        const Squatting& operator = (const Squatting&);

        function<void(const cv::Mat&, const int, std::vector<obs_body_t>&)> poseForward_ = nullptr;
        std::unordered_map<int,std::shared_ptr<SquattingCount>>  SquattingSubMap_;
        std::string app_root_path_;

    };
}


#endif //RKNN_YOLOV5_ANDROID_APK_DEMO_SQUATTING_H
