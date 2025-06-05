//
// Created by dell on 2024/6/11.
//

#ifndef RKNN_HK_V1_RUNTIMINGCOUNT_H
#define RKNN_HK_V1_RUNTIMINGCOUNT_H

#include <iostream>
#include <list>
#include "CommonStruct.h"
using namespace std;

namespace obs {

    class RunTimingCount {

    public:
        RunTimingCount();

        virtual ~RunTimingCount();

        void checkTouchLine(const obs_joint_t &toe_tips_point,const obs_joint_t&heel_point,const int64_t pts,obs_person_t &person);

        void checkJumpGun(const obs_joint_t &toe_tips_point,const obs_joint_t&heel_point,const int64_t pts,obs_person_t &person);

        void checkTouchLine(const std::map<float,obs_foot_t>foots,const int64_t pts,obs_person_t &person);

        void checkJumpGun(const std::map<float,obs_foot_t>foots,const int64_t pts,obs_person_t &person);

        void checkEnding(const int cameraIndex,const obs_joint_t &centerPoint,const int64_t pts,obs_person_t &person);

        void checkEndingReturn(const int cameraIndex,const bool bFirst,const obs_joint_t &centerPoint,const int64_t pts,obs_person_t &person);

        void checkPolePass(const int cameraIndex,const int64_t pts,obs_person_t &person);

        void Reset();

        // 判断认为是否在不同摄像机下，解决50*8条件绕杆未完成时Reset
        void personInCamera(const int cameraIndex);

    public:
        //位置
        int location_ = 0;
        obs_line_t startLine;
        obs_line_t  cheatLine;
        obs_line_t endLine;
        std::vector<obs_joint_t> startRect;
        std::vector<obs_joint_t> endRect;

        //杆坐标
        obs_joint_t polePoint;
        int first_cross_line_x = 0;//第1次过线的x值
        int second_cross_line_x = 0;//第2次过线的x值
        int cross_line_next = 0;

    private:
        bool touchLine_ = false;
        bool bJumpGun_ = false;
        std::list<int> tips_num;

        bool endStatus1 = false;
        int preCheckStatus1 = 0;
        int checkStatus1 = 0;

        bool endStatus2 = false;
        int preCheckStatus2 = 0;
        int checkStatus2 = 0;

        int cameraId = -1;
        bool first_assigned_cameraId = true;
    };
}



#endif //RKNN_HK_V1_RUNTIMINGCOUNT_H
