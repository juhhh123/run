//
// Created by Lige on 2024/4/27.
//

#ifndef RKNN_YOLOV5_ANDROID_APK_DEMO_SQUATTINGCOUNT_H
#define RKNN_YOLOV5_ANDROID_APK_DEMO_SQUATTINGCOUNT_H

#include "CommonStruct.h"
#include <list>

namespace obs {
    class SquattingCount {
    public:
        SquattingCount();
        ~SquattingCount();

        int CountSquatting(int index, const obs_body_t& body);

    public:
        float count_number_ = 0;               //成功次数
        int fail_number_ = 0;                //失败次数
        int pre_count_number_ = 0;
        int interrupt_count_number_ = 0;     //中断次数
        int uid_ = 0;                        //用户ID
        int train_id_ = 0;                   //训练ID
        int location_ = 0;                   //位置
        int out_range_count_ = 0;
        //框坐标点
//        obs_joint_t locationPoint[OBS_SKIP_MAX];
        std::vector<obs_joint_t> locationPoint;
        cv::Rect rect_;

        int interrupt_status_ = 0;           //中断状态
        int pre_interrupt_status_ = 0;
        bool exit_status_ = false;           //出圈状态
        bool enter_in_ = false;              //是否在指定测试区
        bool first_in_ = true;               //
        bool raise_right_hand = false;       //举右手
        bool raise_left_hand = false;        //举左手
        int fail_id_ = 0;
        int fail_count_ = 0;

    private:
        float m_leftshoulderY = 0.0f;
        float m_rightshoulderY = 0.0f;
        float m_lefthipY = 0.0f;
        float m_righthipY = 0.0f;
        float m_leftAnkleY = 0.0f;
        float m_rightAnkleY = 0.0f;

        float m_UpTimes = 0.0f;
        float m_DownTimes = 0.0f;
        float m_disLeftShoulder = 0.0f;
        float m_disRightShoulder = 0.0f;
        std::list<float> m_list_leftShoulder;
        std::list<float> m_list_rightShoulder;
        std::list<float> m_list_leftHip;
        bool isHighest = true;
        bool isLowest = false;
        bool isUpest = true;
        int m_lowTimes = 0;
        int m_highTimes = 0;
        float m_highestShoulder = 0.0f;
        float m_lowestShoulder = 0.0f;
        float m_maxDistanceLeftShouklder_Ankle = 0.0f;
        float m_maxDistanceLeftHip_Ankle = 0.0f;

        int average_value = 10;
        int exit_ = 0;
        int exit_triggered = 0;
        int exit_sub = 1;
        std::list<float> count_number_list;
        int count_no_list = 0;
        std::list<int> num_list;

    };
}


#endif //RKNN_YOLOV5_ANDROID_APK_DEMO_SQUATTINGCOUNT_H
