//
// Created by dell on 2024/6/6.
//

#include "LongRunTiming.h"
#include <fstream>
#include "json.h"
#include "Logger.h"
#include "Flags.h"

#define TIPS_NUM_COUNT 3

namespace obs{

    LongRunTiming::LongRunTiming(const int sportModel,const std::string config_path,const cv::Rect checkRect){
        app_root_path_ = config_path;
        checkRect_ = checkRect;
        const int height_max_y = checkRect.y + checkRect.height;
        std::string running_start_config;
        std::string running_end_config;
        if(sportModel == SPORT_LONG_RUN_TIMING_800){
            running_start_config = config_path + "/config/long_run_timing_start_800.json";
            running_end_config = config_path + "/config/long_run_timing_end_800.json";
        }else if(sportModel == SPORT_LONG_RUN_TIMING_1000){
            running_start_config = config_path + "/config/long_run_timing_start_1000.json";
            running_end_config = config_path + "/config/long_run_timing_end_1000.json";
        }else if(sportModel == SPORT_LONG_RUN_TIMING_3000){
            running_start_config = config_path + "/config/long_run_timing_start_3000.json";
            running_end_config = config_path + "/config/long_run_timing_end_3000.json";
        }else{
            return;
        }

        Json::Reader json_reader;
        Json::Value startRoot;
        Json::Value endRoot;
        ifstream infile(running_start_config.c_str(), ios::binary);
        if(!infile.is_open())
        {
            LogError("Open {%s} failed!",running_start_config);
            return;
        }

        if(!json_reader.parse(infile, startRoot)) {
            LogError("parse {%s} failed!",running_start_config);
            infile.close();
            return;
        }
        infile.close();

        infile.open(running_end_config.c_str(), ios::binary);
        if(!infile.is_open())
        {
            LogError("Open {%s} failed!",running_end_config);
            return;
        }

        if(!json_reader.parse(infile, endRoot)) {
            LogError("parse {%s} failed!",running_end_config);
            infile.close();
            return;
        }
        infile.close();

        Json::Value startLocationList = startRoot["LocationList"];
        for (int i = 0; i < startLocationList.size() - 1; ++i) {
            int x1 = startLocationList[i]["x1"].asInt();
            int y1 = startLocationList[i]["y1"].asInt();
            int x2 = startLocationList[i]["x2"].asInt();
            int y2 = startLocationList[i]["y2"].asInt();

            if (i == 0) {
                startLine_.a = obs_joint_t(x1, y1);
                startLine_.b = obs_joint_t(x2, y2);
                continue;
            }
            int x3 = startLocationList[i + 1]["x1"].asInt();
            int y3 = startLocationList[i + 1]["y1"].asInt();
            int x4 = startLocationList[i + 1]["x2"].asInt();
            int y4 = startLocationList[i + 1]["y2"].asInt();
            startRect.push_back(obs_joint_t(x1,y1));
            startRect.push_back(obs_joint_t(x2,y2));
            if (Flags::checkPointPositionLine(obs_joint_t(x2,y2),startLine_) !=
                Flags::checkPointPositionLine(obs_joint_t(x3,y3),startLine_)){
                startRect.push_back(obs_joint_t(x4,y4));
                startRect.push_back(obs_joint_t(x3,y3));
            }else{
                startRect.push_back(obs_joint_t(x3,y3));
                startRect.push_back(obs_joint_t(x4,y4));
            }
            break;
        }


        Json::Value endLocationList = endRoot["LocationList"];
        for (int i = 0; i < endLocationList.size() - 1; ++i) {

            int x1_ = endLocationList[i]["x1"].asInt();
            int y1_ = endLocationList[i]["y1"].asInt();
            int x2_ = endLocationList[i]["x2"].asInt();
            int y2_ = endLocationList[i]["y2"].asInt();

            if (i == 0){
                cheatLine_.a = obs_joint_t(x1_,y1_);
                cheatLine_.b = obs_joint_t(x2_,y2_);
                continue;
            }

            if (i == 1){
                endLine_.a = obs_joint_t(x1_,y1_);
                endLine_.b = obs_joint_t(x2_,y2_);
                continue;
            }

            int x3_ = endLocationList[i + 1]["x1"].asInt();
            int y3_ = endLocationList[i + 1]["y1"].asInt();
            int x4_ = endLocationList[i + 1]["x2"].asInt();
            int y4_ = endLocationList[i + 1]["y2"].asInt();

            if (y1_ > y2_){
                y1_ = height_max_y;
            }else{
                y2_ = height_max_y;
            }

            if (y3_ > y4_){
                y3_ = height_max_y;
            }else{
                y4_ = height_max_y;
            }

            endRect.push_back(obs_joint_t(x1_,y1_));
            endRect.push_back(obs_joint_t(x2_,y2_));

            if (Flags::checkPointPositionLine(obs_joint_t(x2_,y2_),endLine_) !=
                Flags::checkPointPositionLine(obs_joint_t(x3_,y3_),endLine_)){

                endRect.push_back(obs_joint_t(x4_,y4_));
                endRect.push_back(obs_joint_t(x3_,y3_));
            }else{
                endRect.push_back(obs_joint_t(x3_,y3_));
                endRect.push_back(obs_joint_t(x4_,y4_));
            }

            break;
        }

//        Json::Value faceRectList = endRoot["RegFaceRectList"];
//        for (int i = 0; i < faceRectList.size(); ++i){
//            Json::Value rectPointList = faceRectList[i]["RectPoint"];
//            for (int i = 0; i < rectPointList.size(); ++i) {
//                int x = rectPointList[i]["x"].asInt();
//                int y = rectPointList[i]["y"].asInt();
//                regFaceRect_.push_back(obs_joint_t(x,y));
//            }
//        }

    }

    LongRunTiming::~LongRunTiming(){

    }

    void LongRunTiming::Execute(const int cameraIndex,const cv::Mat &frame,const int location, const int64_t pts,std::vector<obs_person_t> &persons){

        if(segForward_ == nullptr || recognitionFace_ == nullptr || personForward_== nullptr ||frame.empty()){
            return;
        }

        std::lock_guard<std::mutex>lock(mtx_);
        if(bStatus_ == START_STATUS){
            obs_person_t person;
            person.cameraIndex = cameraIndex;
            person.id = 1;
            person.msec = pts;
            person.over_flag = START_STATUS;
            persons.push_back(person);
            bStatus_ = CHECK_ENDING;
            return;
        }

        if(cameraIndex == 0){

            std::vector<obs_mask_t> segMask;
            segForward_(frame,location,segMask);

            if (segMask.empty()) return;

            obs_person_t person;
            person.cameraIndex = cameraIndex;

            if(bStatus_ == CHECK_TOUCH_LINE){

                checkTouchLine(segMask,pts,person);

            }else if (bStatus_ == CHECK_JUNP_GUN){

                checkJumpGun(segMask,pts,person);
            }
            persons.push_back(person);

            if (!longRunnerMap_.empty()){
                longRunnerMap_.clear();
            }

        }else{

            std::vector<obs_persons_box_t> personsRect;
            personForward_(frame,location,personsRect);
            for (const auto& personRect : personsRect){

                obs_joint_t point = obs_joint_t(personRect.bodyRect.x + personRect.bodyRect.width/2.0,personRect.bodyRect.y + personRect.bodyRect.height);

                //撞线区域
                if(!Flags::isPointInPolygon(endRect,point)){
                    continue;
                }

                obs_person_t person;
                person.cameraIndex = cameraIndex;
                person.id = personRect.id;
                person.rect = personRect.bodyRect;
                checkEnding(personRect,frame,point,pts,person);
                persons.push_back(person);
            }
        }
    }

    void LongRunTiming::checkTouchLine(const std::vector<obs_mask_t> &segMask,const int64_t pts,obs_person_t &person){

        bool touch = false;
        person.id = 1;
        for (int i = 0; i < segMask.size(); ++i) {

            const obs_mask_t &obsMask = segMask.at(i);
            const obs_joint_t checkPoint = obs_joint_t(obsMask.rect.x + obsMask.rect.width/2.0,obsMask.rect.y + obsMask.rect.height);
            if(!Flags::isPointInPolygon(startRect,checkPoint)){
                continue;
            }

            float start_min_height_y = obsMask.rect.y + 9*(obsMask.rect.height) / 10.0;
            obs_joint_t toe_tips_point,heel_point;
            getToeTipsPoint(obsMask.masks,start_min_height_y,toe_tips_point,heel_point);

            person.rect = obsMask.rect;
            obs_line_t footLine;
            footLine.a = heel_point;
            footLine.b =  toe_tips_point;
            footLine.b.x = footLine.b.x - 6;
            if(Flags::distanceToeTipsCal(footLine,startLine_)){
                touch = true;
                break;
            }
        }

        if (touch && !bTouchLine_){
            person.over_flag = NOT_READY;
            person.fail_id = RUNNING_JUMP_TOUCH_LINE;
            person.msec = pts;
            no_touch_num_ = 0;
            bTouchLine_ = true;
            LOGE("踩线,pts-->%ld",pts);
        }else{
            no_touch_num_ ++;
        }

        if (no_touch_num_ > TIPS_NUM_COUNT) {
            person.over_flag = READY_STATUS;
            person.fail_id = SPORT_NONE_ERROR;
            bTouchLine_ = false;
            LOGE("ready");
        }
    }

    void LongRunTiming::checkJumpGun(const std::vector<obs_mask_t> &segMask,const int64_t pts,obs_person_t &person){

        bool jumpGun = false;
        person.id = 1;
        for (int i = 0; i < segMask.size(); ++i) {
            const obs_mask_t &obsMask = segMask.at(i);
            const obs_joint_t checkPoint = obs_joint_t(obsMask.rect.x + obsMask.rect.width/2.0,obsMask.rect.y + obsMask.rect.height);
            if(!Flags::isPointInPolygon(startRect,checkPoint)){
                continue;
            }

            float start_min_height_y = obsMask.rect.y + 9*(obsMask.rect.height) / 10.0;
            obs_joint_t toe_tips_point,heel_point;
            getToeTipsPoint(obsMask.masks,start_min_height_y,toe_tips_point,heel_point);

            obs_line_t footLine;
            footLine.a = heel_point;
            footLine.b =  toe_tips_point;
            footLine.b.x = footLine.b.x - 6;

            if(Flags::distanceToeTipsCal(footLine,startLine_) || Flags::checkPointPosition(checkPoint,startLine_.a,startLine_.b) > 0){
                jumpGun = true;
                break;
            }
        }

        if (jumpGun && !bJumpGun_){
            person.over_flag = NOT_READY;
            person.fail_id = RUNNING_JUMP_THE_GUN;
            person.msec = pts;
            no_touch_num_ = 0;
            bJumpGun_ = true;
            LogInfo("抢跑");
            LOGE("抢跑");
//            LOGE("pts-->%ld",pts);
        }else{
            person.over_flag = READY_STATUS;
            person.fail_id = SPORT_NONE_ERROR;
            bJumpGun_ = false;
            LogInfo("ready");
            LOGE("ready");
        }
    }

    void LongRunTiming::checkEnding(const obs_body_t &body,const cv::Mat &frame,const obs_joint_t &point,const int64_t pts,obs_person_t &person){

        if (longRunnerMap_.find(body.id) == longRunnerMap_.end()){
            obs_runner_t obsUser;
            obsUser.personId = body.id;
            longRunnerMap_[body.id] = obsUser;
        }

        obs_joint_t left_shoulder = body.joints[OBS_JOINT_SHOULDER_LEFT];
        obs_joint_t right_shoulder = body.joints[OBS_JOINT_SHOULDER_RIGHT];
        obs_joint_t neck = body.joints[OBS_JOINT_NECK];
        if (Flags::is_point_valid(left_shoulder) && Flags::is_point_valid(right_shoulder) && Flags::is_point_valid(neck)){

            obs_runner_t &obsRunner = longRunnerMap_[body.id];
            if (++obsRunner.recog_face_rate_ > RUNNING_RECOGNITION_FACE_RATE){
                int width = std::abs(left_shoulder.x - right_shoulder.x);
                float rectY = body.rect.y;
                cv::Rect recgRect;
                if (rectY * DIFFEN_FACE_MIN_HEIGHT > checkRect_.y){
                    rectY = rectY * DIFFEN_FACE_MIN_HEIGHT;
                }else{
                    rectY = checkRect_.y;
                }

                recgRect.x = neck.x - (width * DIFFEN_FACE_WIDTH / 2) - checkRect_.x;
                recgRect.y = rectY - checkRect_.y;
                recgRect.width = width * DIFFEN_FACE_WIDTH;
                recgRect.height = std::abs(neck.y - rectY);

                if (recgRect.area() > 100.0f){
                    recgRect = recgRect & cv::Rect(0, 0, frame.cols, frame.rows);
                    cv::Mat bodyImg = frame(recgRect);
                    if (recognitionFace_){
                        recognitionFace_(bodyImg,1,body.id,neck.x,rectY);
                    }
                }
                obsRunner.recog_face_rate_ = 0;
            }
        }

        if (!Flags::is_point_valid(point)) return;

        if(Flags::checkPointPosition(point,endLine_.a,endLine_.b) > 0){
            longRunnerMap_[body.id].checkStatus = 2;
        }else{
            longRunnerMap_[body.id].checkStatus = 1;
        }

        if(longRunnerMap_[body.id].preCheckStatus == 1){
            if(longRunnerMap_[body.id].checkStatus == 2){
                longRunnerMap_[body.id].endStatus = true;
            }
        }

        longRunnerMap_[body.id].preCheckStatus = longRunnerMap_[body.id].checkStatus;

        if(longRunnerMap_[body.id].endStatus){
            person.over_flag = END_STATUS;
            person.msec = pts;
            longRunnerMap_[body.id].endStatus = false;
            LOGE("BodyRect.id: %d, over_flag: %d\n",body.id,person.over_flag);
        }
    }

    void LongRunTiming::checkEnding(const obs_persons_box_t &personRect,const cv::Mat &frame,const obs_joint_t &point,const int64_t pts,obs_person_t &person){

        if (longRunnerMap_.find(personRect.id) == longRunnerMap_.end()){
            obs_runner_t obsUser;
            obsUser.personId = personRect.id;
            longRunnerMap_[personRect.id] = obsUser;
        }

        obs_runner_t &obsRunner = longRunnerMap_[personRect.id];
        if (!obsRunner.isRecog && ++obsRunner.recog_face_rate_ > RUNNING_RECOGNITION_FACE_RATE){
            cv::Rect recgRect = personRect.headRect;
            recgRect.x = personRect.headRect.x - checkRect_.x;
            recgRect.y = personRect.headRect.y - checkRect_.y;
            if (recgRect.area() > 100.0f){
                recgRect = recgRect & cv::Rect(0, 0, frame.cols, frame.rows);
                cv::Mat bodyImg = frame(recgRect);
                if (recognitionFace_){
                    recognitionFace_(bodyImg,1,personRect.id,personRect.headRect.x,personRect.headRect.y);
                }
            }
            obsRunner.recog_face_rate_ = 0;
        }

        if(Flags::checkPointPosition(point,endLine_.a,endLine_.b) > 0){
            longRunnerMap_[personRect.id].checkStatus = 2;
        }else{
            longRunnerMap_[personRect.id].checkStatus = 1;
        }

        if(longRunnerMap_[personRect.id].preCheckStatus == 1){
            if(longRunnerMap_[personRect.id].checkStatus == 2){
                longRunnerMap_[personRect.id].endStatus = true;
            }
        }

        longRunnerMap_[personRect.id].preCheckStatus = longRunnerMap_[personRect.id].checkStatus;

        if(longRunnerMap_[personRect.id].endStatus){
            person.over_flag = END_STATUS;
            person.msec = pts;
            longRunnerMap_[personRect.id].endStatus = false;
            LOGE("BodyRect.id: %d, over_flag: %d\n",personRect.id,person.over_flag);
        }

    }

    void LongRunTiming::getToeTipsPoint(const std::vector<cv::Point2f> &masks,const float start_min_height_y,obs_joint_t &toe_tips_point,obs_joint_t &heel_point){

        float maxX = 0.0f;
        float minX = 10000.0f;
        for (int i = 0; i < masks.size(); ++i) {
            if(masks[i].x < EPISON || masks[i].y < EPISON || masks[i].y < start_min_height_y){
                continue;
            }

            if (masks[i].x > maxX) {//脚尖
                maxX = masks[i].x;
                toe_tips_point = obs_joint_t(masks[i].x,masks[i].y);
            }

            if (masks[i].x < minX) {//脚后跟
                minX = masks[i].x;
                heel_point = obs_joint_t(masks[i].x,masks[i].y);
            }
        }

//        for (int i = 0; i < masks.size(); ++i) {
//            if(masks[i].x < EPISON || masks[i].y < EPISON || masks[i].y < start_min_height_y){
//                continue;
//            }
//
//            if (masks[i].x > maxX) {//脚后跟
//                maxX = masks[i].x;
//                heel_point = obs_joint_t(masks[i].x,masks[i].y);
//            }
//
//            if (masks[i].x < minX) {//脚尖
//                minX = masks[i].x;
//                toe_tips_point = obs_joint_t(masks[i].x,masks[i].y);
//            }
//        }

    }

}