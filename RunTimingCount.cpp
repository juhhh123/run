//
// Created by dell on 2024/6/11.
//

#include "RunTimingCount.h"
#include "Flags.h"
#include "Logger.h"

namespace obs {

    RunTimingCount::RunTimingCount(){

    }

    RunTimingCount:: ~RunTimingCount(){

    }

    void RunTimingCount::checkTouchLine(const obs_joint_t &toe_tips_point,const obs_joint_t&heel_point,const int64_t pts,obs_person_t &person){

        obs_line_t footLine;
        footLine.a = heel_point;
        footLine.b =  toe_tips_point;

//        if(Flags::distanceToeTipsCal(footLine,startLine)){

        if(Flags::checkPointPosition(toe_tips_point,startLine.a,startLine.b) > 0){
//            tips_num.push_back(1);
        }else{
//            tips_num.push_back(0);
        }

        if(tips_num.size() >= 3){
            tips_num.pop_front();
        }

        if(tips_num.size() == 2){
            auto min_it = std::min_element(tips_num.begin(), tips_num.end());
            if(*min_it == 1) {
                person.over_flag = NOT_READY;
                person.fail_id = RUNNING_JUMP_TOUCH_LINE;
                person.msec = pts;
//                touchLine_ = true;
                LOGE("location-->{%d} 踩线 pts-->%ld",location_,pts);
            }

            auto max_it = std::max_element(tips_num.begin(), tips_num.end());
            if(*max_it == 0){
                person.over_flag = READY_STATUS;
                person.fail_id = SPORT_NONE_ERROR;
//                touchLine_ = false;
                LOGE("location-->{%d} ready pts-->%ld",location_,pts);
            }
        }
    }

    void RunTimingCount::checkJumpGun(const obs_joint_t &toe_tips_point,const obs_joint_t&heel_point,const int64_t pts,obs_person_t &person){

//        obs_line_t footLine;
//        footLine.a = heel_point;
//        footLine.b =  toe_tips_point;
//        footLine.b.x = footLine.b.x - 12;
//        if(Flags::distanceToeTipsCal(footLine,startLine)){
//            tips_num.push_back(1);
//        }else{
//            tips_num.push_back(0);
//        }
//
//        if(tips_num.size() >= 3){
//            tips_num.pop_front();
//        }
//
//        if(tips_num.size() == 2) {
//            auto min_it = std::min_element(tips_num.begin(), tips_num.end());
//            if (*min_it == 1 && !bJumpGun_) {
//                person.over_flag = NOT_READY;
//                person.fail_id = RUNNING_JUMP_THE_GUN;
//                person.msec = pts;
//                bJumpGun_ = true;
//                LOGE("location-->{%d} 抢跑 pts-->%ld",location_,pts);
//                return;
//            }
//        }

        if(Flags::checkPointPosition(heel_point,startLine.a,startLine.b) > 0 && !bJumpGun_){
            person.over_flag = NOT_READY;
            person.fail_id = RUNNING_JUMP_THE_GUN;
            person.msec = pts;
            bJumpGun_ = true;
            LOGE("location-->{%d} 抢跑 pts-->%ld",location_,pts);
        }else{
            person.over_flag = READY_STATUS;
            person.fail_id = SPORT_NONE_ERROR;
            bJumpGun_ = false;
            LOGE("location-->{%d} ready",location_);
        }
    }

    void RunTimingCount::checkTouchLine(const std::map<float,obs_foot_t>foots,const int64_t pts,obs_person_t &person){

        if (foots.empty()) return;

        const int nSize = foots.size();
        if (nSize == 1)
        {
            auto foot1 = foots.begin();
            const obs_joint_t toe_tips_point_1 = obs_joint_t(foot1->second.rect.x,foot1->second.rect.y + foot1->second.rect.height);
            if(Flags::checkPointPosition(toe_tips_point_1,startLine.a,startLine.b) > 0){
                tips_num.push_back(1);
            }else{
                tips_num.push_back(0);
            }

            person.rect = foot1->second.rect;

        }else if(nSize == 2){
            auto foot1 = foots.begin();
            auto foot2 = foots.rbegin();
            const obs_joint_t toe_tips_point_1 = obs_joint_t(foot1->second.rect.x,foot1->second.rect.y + foot1->second.rect.height);
            const obs_joint_t toe_tips_point_2 = obs_joint_t(foot2->second.rect.x,foot2->second.rect.y + foot2->second.rect.height);
            const obs_joint_t heel_point_1 = obs_joint_t(foot1->second.rect.x + foot1->second.rect.width,foot1->second.rect.y + foot1->second.rect.height);
            const obs_joint_t heel_point_2 = obs_joint_t(foot2->second.rect.x + + foot2->second.rect.width,foot2->second.rect.y + foot2->second.rect.height);

            if(Flags::checkPointPosition(toe_tips_point_1,startLine.a,startLine.b) > 0 || Flags::checkPointPosition(toe_tips_point_2,startLine.a,startLine.b) > 0){
                tips_num.push_back(1);
            }else{
                tips_num.push_back(0);
            }

            person.rect = foot1->second.rect;
        }

        if(tips_num.size() >= 3){
            tips_num.pop_front();
        }

        if(tips_num.size() == 2){
            auto min_it = std::min_element(tips_num.begin(), tips_num.end());
            if(*min_it == 1 && !touchLine_) {
                person.over_flag = NOT_READY;
                person.fail_id = RUNNING_JUMP_TOUCH_LINE;
                person.msec = pts;
                touchLine_ = true;
                LOGE("location-->{%d} 踩线 pts-->%ld",location_,pts);
            }

            auto max_it = std::max_element(tips_num.begin(), tips_num.end());
            if(*max_it == 0){
                person.over_flag = READY_STATUS;
                person.fail_id = SPORT_NONE_ERROR;
                touchLine_ = false;
                LOGE("location-->{%d} ready pts-->%ld",location_,pts);
            }
        }
    }

    void RunTimingCount::checkJumpGun(const std::map<float,obs_foot_t>foots,const int64_t pts,obs_person_t &person){

        if (foots.empty()) return;
        const int nSize = foots.size();
        if (nSize == 1)
        {
            auto foot1 = foots.begin();
            obs_joint_t heel_point_1 = obs_joint_t(foot1->second.rect.x + foot1->second.rect.width,foot1->second.rect.y + foot1->second.rect.height);

            if(Flags::checkPointPosition(heel_point_1,startLine.a,startLine.b) > 0 && !bJumpGun_){
                person.over_flag = NOT_READY;
                person.fail_id = RUNNING_JUMP_THE_GUN;
                person.msec = pts;
                bJumpGun_ = true;
                LOGE("location-->{%d} 抢跑 pts-->%ld",location_,pts);
            }else{
                person.over_flag = READY_STATUS;
                person.fail_id = SPORT_NONE_ERROR;
                bJumpGun_ = false;
                LOGE("location-->{%d} ready",location_);
            }

        }else if(nSize == 2){

            auto foot1 = foots.begin();
            auto foot2 = foots.rbegin();
            obs_joint_t heel_point_1 = obs_joint_t(foot1->second.rect.x + foot1->second.rect.width,foot1->second.rect.y + foot1->second.rect.height);
            obs_joint_t heel_point_2 = obs_joint_t(foot2->second.rect.x + + foot2->second.rect.width,foot2->second.rect.y + foot2->second.rect.height);

            if((Flags::checkPointPosition(heel_point_1,startLine.a,startLine.b) > 0 || Flags::checkPointPosition(heel_point_2,startLine.a,startLine.b) > 0) && !bJumpGun_){
                person.over_flag = NOT_READY;
                person.fail_id = RUNNING_JUMP_THE_GUN;
                person.msec = pts;
                bJumpGun_ = true;
                LOGE("location-->{%d} 抢跑 pts-->%ld",location_,pts);
            }else{
                person.over_flag = READY_STATUS;
                person.fail_id = SPORT_NONE_ERROR;
                bJumpGun_ = false;
                LOGE("location-->{%d} ready",location_);
            }

        }
    }

    void RunTimingCount::checkEnding(const int cameraIndex,const obs_joint_t &centerPoint,const int64_t pts,obs_person_t &person){

        if(Flags::checkPointPosition(centerPoint,endLine.a,endLine.b) > 0){
            checkStatus1 = 2;
        }else{
            checkStatus1 = 1;
        }

        if(preCheckStatus1 == 1){
            if(checkStatus1 == 2){
                endStatus1 = true;
            }
        }

        preCheckStatus1 = checkStatus1;

        if (endStatus1){
            person.over_flag = END_STATUS;
            person.msec = pts;
            endStatus1 = false;
            LOGE("cameraIndex-->{%d},location-->{%d} 撞线\n",cameraIndex,location_);
        }
    }

    void RunTimingCount::checkEndingReturn(const int cameraIndex,const bool bFirst,const obs_joint_t &centerPoint,const int64_t pts,obs_person_t &person){

        if (bFirst){
            if(Flags::checkPointPosition(centerPoint,endLine.a,endLine.b) >= 0){
                checkStatus1 = 2;//直线下方
            }else{
                checkStatus1 = 1;//直线上方
            }

            if(preCheckStatus1 == 1){
                if(checkStatus1 == 2){
                    endStatus1 = true;
                }
            }
            preCheckStatus1 = checkStatus1;

            if (endStatus1){
                person.over_flag = END_STATUS_1;
                person.msec = pts;
                endStatus1 = false;
                first_cross_line_x = (int)centerPoint.x;
                LOGE("cameraIndex-->{%d},location-->{%d},cross_line_x-->{%d},pole_x-->{%f},END_STATUS_1 撞线\n",cameraIndex,location_,first_cross_line_x,polePoint.x);
            }

        }else{

            if(Flags::checkPointPosition(centerPoint,endLine.a,endLine.b) >= 0){
                checkStatus2 = 2;//直线下方
            }else{
                checkStatus2 = 1;//直线上方
            }

            if(preCheckStatus2 == 2){
                if(checkStatus2 == 1){
                    endStatus2 = true;
                }
            }
            preCheckStatus2 = checkStatus2;

            if (endStatus2){
                person.over_flag = END_STATUS_2;
                person.msec = pts;
                endStatus2 = false;
                second_cross_line_x = (int)centerPoint.x;
                LOGE("cameraIndex-->{%d},location-->{%d},cross_line_x-->{%d}, pole_x-->{%f},END_STATUS_2 撞线\n",cameraIndex,location_,second_cross_line_x,polePoint.x);
            }
        }
    }

    //杆的坐标x值，在两次过线x值之间，满足绕杆
    void RunTimingCount::checkPolePass(const int cameraIndex,const int64_t pts,obs_person_t &person){
        if ((polePoint.x >= first_cross_line_x && polePoint.x <= second_cross_line_x) || (polePoint.x >= second_cross_line_x  && polePoint.x <= first_cross_line_x)){
            person.over_flag = ROUND_POLE_OK;
            person.msec = pts;
            LOGE("cameraIndex-->{%d},location-->{%d},ROUND_POLE_OK",cameraIndex,location_);
        }else{
            person.fail_id = ROUND_POLE_FAIL;
            person.msec = pts;
            LOGE("cameraIndex-->{%d},location-->{%d},ROUND_POLE_FAIL",cameraIndex,location_);
        }
    }

    void RunTimingCount::Reset(){
        first_cross_line_x = 0;
        second_cross_line_x = 0;
        endStatus1 = false;
        preCheckStatus1 = 0;
        checkStatus1 = 0;
        endStatus2 = false;
        preCheckStatus2 = 0;
        checkStatus2 = 0;
        cross_line_next = 0;
    }

    void RunTimingCount::personInCamera(const int cameraIndex)
    {
        if (first_assigned_cameraId) {
            cameraId = cameraIndex;
            first_assigned_cameraId = false;
        }

        if (cameraId != cameraIndex) {
            if (first_cross_line_x != 0 || second_cross_line_x != 0 || cross_line_next == 2) {
                Reset();
            }
            cameraId = cameraIndex;
        }
    }

}
