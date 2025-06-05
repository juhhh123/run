//
// Created by dell on 2024/4/19.
//

#ifndef RKNN_YOLOV5_ANDROID_APK_DEMO_LOCATIONPOINT_H
#define RKNN_YOLOV5_ANDROID_APK_DEMO_LOCATIONPOINT_H

#include "TimesUtil.h"
#include <iostream>
#include <vector>
#include "CommonStruct.h"

using namespace std;

namespace obs {

    class LocationPoint {
    public:

        LocationPoint();
        ~LocationPoint();

        void clear();

        void reset();

        void startTimer(uint64_t timeOut);

        bool isTimeOut();

        int location = 0;

        bool b_first = false;

        bool enter_in_ = false;

        int64_t out_range_count_ = 0;

        std::vector<obs_joint_t> localRect;

        int recog_rate_ = 0;

    private:
        uint64_t startTime_ = 0;
        uint64_t timeOut_ = 0;
    };

}


#endif //RKNN_YOLOV5_ANDROID_APK_DEMO_LOCATIONPOINT_H
