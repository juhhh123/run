//
// Created by dell on 2024/6/6.
//

#include "ShortRunTiming.h"
#include <fstream>
#include "json.h"
#include "Logger.h"
#include "Flags.h"
#include "TimesUtil.h"

namespace obs{

    ShortRunTiming::ShortRunTiming(const int sportModel,const std::string config_path,const int cameraIndex){

        obs_config_path_ = config_path;
        sportModel_ = sportModel;
        std::string running_start_config;
        std::string running_end1_config;
        std::string running_end2_config;

        if(sportModel == SPORT_SHORT_RUN_TIMING_50){
            running_start_config = config_path + "/config/short_run_timing_start_50.json";
            running_end1_config = config_path + "/config/short_run_timing_end_50.json";
        }else if(sportModel == SPORT_SHORT_RUN_TIMING_60){
            running_start_config = config_path + "/config/short_run_timing_start_60.json";
            running_end1_config = config_path + "/config/short_run_timing_end_60.json";
        }else if(sportModel == SPORT_SHORT_RUN_TIMING_100){
            running_start_config = config_path + "/config/short_run_timing_start_100.json";
            running_end1_config = config_path + "/config/short_run_timing_end_100.json";
        }else if (sportModel == SPORT_SHORT_RUN_TIMING_8x50){
            running_start_config = config_path + "/config/short_run_timing_start_8x50.json";
            running_end1_config = config_path + "/config/short_run_timing_end_8x50_1.json";
            running_end2_config = config_path + "/config/short_run_timing_end_8x50_2.json";
        }else if(sportModel == SPORT_BASKET_BALL_ROUND){
            running_start_config = config_path + "/config/basket_ball_round_start.json";
            running_end1_config = config_path + "/config/basket_ball_round_end.json";
        }else if(sportModel == SPORT_FOOT_BALL_ROUND){
            running_start_config = config_path + "/config/foot_ball_round_start.json";
            running_end1_config = config_path + "/config/foot_ball_round_end.json";
        }else{
            return;
        }

        Json::Reader json_reader;
        Json::Value startRoot;
        ifstream infile(running_start_config.c_str(), ios::binary);
        if(!infile.is_open())
        {
            LogError("Open {} failed!",running_start_config);
            return;
        }

        if(!json_reader.parse(infile, startRoot)) {
            LogError("parse {} failed!",running_start_config);
            infile.close();
            return;
        }
        infile.close();

        obs_line_t startLine;
        Json::Value startLocationList = startRoot["LocationList"];
        const int jsSize = startLocationList.size();
        midRunway = jsSize / 2 -1;
        LOGE("midRunway:%d",midRunway);
        for (int i = 0; i < jsSize - 1; ++i) {
            int x1 = startLocationList[i]["x1"].asInt();
            int y1 = startLocationList[i]["y1"].asInt();
            int x2 = startLocationList[i]["x2"].asInt();
            int y2 = startLocationList[i]["y2"].asInt();

            if (i == 0){
                startLine.a = obs_joint_t(x1,y1);
                startLine.b = obs_joint_t(x2,y2);
                continue;
            }

            int x3 = startLocationList[i + 1]["x1"].asInt();
            int y3 = startLocationList[i + 1]["y1"].asInt();
            int x4 = startLocationList[i + 1]["x2"].asInt();
            int y4 = startLocationList[i + 1]["y2"].asInt();

            obs_joint_t startInter1,startInter2;
            obs_line_t currentStartLine = obs_line_t(obs_joint_t(x1,y1),obs_joint_t(x2,y2));
            obs_line_t nextStartLine = obs_line_t(obs_joint_t(x3,y3),obs_joint_t(x4,y4));
            Flags::getIntersectionPoint(currentStartLine,startLine,startInter1);
            Flags::getIntersectionPoint(nextStartLine,startLine,startInter2);

            std::shared_ptr<RunTimingCount> runTimingCount(new RunTimingCount());
            runTimingCount->location_ = i;
            runTimingCount->startLine = obs_line_t(startInter1,startInter2);
            runTimingCount->startRect.push_back(obs_joint_t(x1,y1));
            runTimingCount->startRect.push_back(obs_joint_t(x2,y2));
            if (Flags::checkPointPositionLine(obs_joint_t(x2,y2),startLine) !=
                Flags::checkPointPositionLine(obs_joint_t(x3,y3),startLine)){
                runTimingCount->startRect.push_back(obs_joint_t(x4,y4));
                runTimingCount->startRect.push_back(obs_joint_t(x3,y3));
            }else{
                runTimingCount->startRect.push_back(obs_joint_t(x3,y3));
                runTimingCount->startRect.push_back(obs_joint_t(x4,y4));
            }

            runTimingCountMap_[runTimingCount->location_] = runTimingCount;
        }

        Json::Value endRoot;
        if(sportModel == SPORT_SHORT_RUN_TIMING_8x50 && cameraIndex == 2){
            infile.open(running_end2_config.c_str(), ios::binary);
        }else{
            infile.open(running_end1_config.c_str(), ios::binary);
        }

        if(!infile.is_open())
        {
            LogError("Open {} failed!",running_end1_config);
            return;
        }
        if(!json_reader.parse(infile, endRoot)) {
            LogError("parse {} failed!",running_end1_config);
            infile.close();
            return;
        }
        infile.close();

        obs_line_t cheatLine,endLine;
        Json::Value end1LocationList = endRoot["LocationList"];
        for (int i = 0; i < end1LocationList.size() - 1; ++i) {
            int x1_ = end1LocationList[i]["x1"].asInt();
            int y1_ = end1LocationList[i]["y1"].asInt();
            int x2_ = end1LocationList[i]["x2"].asInt();
            int y2_ = end1LocationList[i]["y2"].asInt();

            if (i == 0){
                cheatLine.a = obs_joint_t(x1_,y1_);
                cheatLine.b = obs_joint_t(x2_,y2_);
                continue;
            }
            if (i == 1){
                endLine.a = obs_joint_t(x1_,y1_);
                endLine.b = obs_joint_t(x2_,y2_);
                continue;
            }

            int x3_ = end1LocationList[i + 1]["x1"].asInt();
            int y3_ = end1LocationList[i + 1]["y1"].asInt();
            int x4_ = end1LocationList[i + 1]["x2"].asInt();
            int y4_ = end1LocationList[i + 1]["y2"].asInt();

            obs_joint_t cheatInter1,cheatInter2;
            obs_joint_t endInter1,endInter2;
            obs_line_t currentEndLine = obs_line_t(obs_joint_t(x1_,y1_),obs_joint_t(x2_,y2_));
            obs_line_t nextEndLine = obs_line_t(obs_joint_t(x3_,y3_),obs_joint_t(x4_,y4_));
            Flags::getIntersectionPoint(currentEndLine,cheatLine,cheatInter1);
            Flags::getIntersectionPoint(nextEndLine,cheatLine,cheatInter2);
            Flags::getIntersectionPoint(currentEndLine,endLine,endInter1);
            Flags::getIntersectionPoint(nextEndLine,endLine,endInter2);

            if (runTimingCountMap_.find(i -1) != runTimingCountMap_.end()){
                runTimingCountMap_[i -1]->cheatLine = obs_line_t(cheatInter1,cheatInter2);
                runTimingCountMap_[i -1]->endLine = obs_line_t(endInter1,endInter2);
                runTimingCountMap_[i -1]->endRect.push_back(obs_joint_t(x1_,y1_));
                runTimingCountMap_[i -1]->endRect.push_back(obs_joint_t(x2_,y2_));
                if (Flags::checkPointPositionLine(obs_joint_t(x2_,y2_),endLine) !=
                    Flags::checkPointPositionLine(obs_joint_t(x3_,y3_),endLine)){

                    runTimingCountMap_[i -1]->endRect.push_back(obs_joint_t(x4_,y4_));
                    runTimingCountMap_[i -1]->endRect.push_back(obs_joint_t(x3_,y3_));
                }else{
                    runTimingCountMap_[i -1]->endRect.push_back(obs_joint_t(x3_,y3_));
                    runTimingCountMap_[i -1]->endRect.push_back(obs_joint_t(x4_,y4_));
                }
                //杆
                runTimingCountMap_[i -1]->polePoint = obs_joint_t((endInter1.x + endInter2.x)/2.0,(endInter1.y + endInter2.y)/2.0);
            }
        }
    }

    ShortRunTiming:: ~ShortRunTiming(){

    }

    void ShortRunTiming::Execute(const int cameraIndex,const cv::Mat &frame,const int location, const int64_t pts,std::vector<obs_person_t> &persons){

        if(personForward_ == nullptr || segForward_ == nullptr || footForward_ == nullptr ||  frame.empty()){
            return;
        }

        std::vector<obs_persons_box_t> personsRect;
        std::vector<obs_foot_t> foots;
        if (cameraIndex == 0){
            footForward_(frame,location,foots);
            if (foots.empty()) {
                obs_person_t person;
                person.cameraIndex = cameraIndex;
                person.id = 1;
                person.msec = pts;
                persons.push_back(person);
                return;
            }

        }else{//1,2

            personForward_(frame,location,personsRect);
            if (personsRect.empty()) {
//                LOGE("personsRect.empty");
                return;
            }
        }

        {
            std::lock_guard<std::mutex>lock(mtx_);
            if (bStatus_ == START_STATUS){
                obs_person_t person;
                person.cameraIndex = cameraIndex;
                person.id = 1;
                person.msec = pts;
                person.over_flag = START_STATUS;
                persons.push_back(person);
                bStatus_ = -1;
                return;
            }
        }

        for (auto iter = runTimingCountMap_.begin(); iter != runTimingCountMap_.end();iter++) {

            std::shared_ptr<RunTimingCount> obj = iter->second;

            obs_person_t person;
            person.id = obj->location_;
            person.cameraIndex = cameraIndex;

            if(cameraIndex == 0){//0摄像头 检测踩线、抢跑

                if (location == 1){
                    if (obj->location_ > midRunway) continue;
                }else if(location == 2){
                    if (obj->location_ <= midRunway) continue;
                }

                //拿到跑道的脚框
                std::map<float,obs_foot_t>selectFoots;
                for (int i = 0; i < foots.size(); ++i) {
                    const obs_foot_t &foot = foots.at(i);
                    obs_joint_t toe_tips_point = obs_joint_t(foot.rect.x,foot.rect.y + foot.rect.height);
                    if (Flags::isPointInPolygon(obj->startRect,toe_tips_point)){
                        float dis = Flags::pointToLineDistance(toe_tips_point,obj->startLine.a,obj->startLine.b);
                        selectFoots[dis] = foot;
                    }
                }

                //删除多余的框
                if (selectFoots.size() > 2){
                    auto it = selectFoots.begin();
                    std::advance(it, 2);
                    selectFoots.erase(it, selectFoots.end());
                }

                {
                    std::lock_guard<std::mutex>lock(mtx_);
                    if(bStatus_ == CHECK_TOUCH_LINE){//踩线
                        obj->checkTouchLine(selectFoots,pts,person);
                    }else if (bStatus_ == CHECK_JUNP_GUN){//抢跑
                        obj->checkJumpGun(selectFoots,pts,person);
                    }
                }

            }else{//1、2摄像头 撞线

                {
                    std::lock_guard<std::mutex>lock(mtx_);
                    if (bStatus_ != CHECK_ENDING) return;
                }

                std::map<float,obs_persons_box_t>matchRectMap;
                for (const auto& personRect : personsRect){

                    if (personRect.bodyRect.x <= 0 || personRect.bodyRect.y <= 0){
                        continue;
                    }

                    obs_joint_t point = obs_joint_t(personRect.bodyRect.x + personRect.bodyRect.width/2.0,personRect.bodyRect.y + personRect.bodyRect.height);

                    if (!Flags::isPointInPolygon(obj->endRect,point)){
                        continue;
                    }

                    matchRectMap[personRect.bodyRect.area()] = personRect;
                }

                auto bodyIter = matchRectMap.rbegin();
                if (bodyIter == matchRectMap.rend()) {
                    continue;
                }

                const obs_persons_box_t &objBody = bodyIter->second;
                person.rect = objBody.bodyRect;
                //框下边缘中点
                obs_joint_t point = obs_joint_t(objBody.bodyRect.x + objBody.bodyRect.width/2.0,objBody.bodyRect.y + objBody.bodyRect.height);

                if (sportModel_ == SPORT_SHORT_RUN_TIMING_8x50){

                    obj->personInCamera(cameraIndex);

                    //过线1
                    if (obj->first_cross_line_x == 0){
                        obj->checkEndingReturn(cameraIndex,true,point,pts,person);
                    }

                    //过线2
                    if (obj->first_cross_line_x != 0 && obj->second_cross_line_x == 0){
                        obj->checkEndingReturn(cameraIndex,false,point,pts,person);
                    }

                    //2次过线后，判断是否完成绕杆
                    if (obj->first_cross_line_x != 0 && obj->second_cross_line_x != 0){
                        if (++obj->cross_line_next == 2){//下一帧进行判断绕杆
                            obj->checkPolePass(cameraIndex,pts,person);
                        }
                    }

                    if (obj->cross_line_next == 2){
                        obj->Reset();
                    }

                }else{
                    obj->checkEnding(cameraIndex,point,pts,person);
                }
            }

            persons.push_back(person);
        }
    }

    void ShortRunTiming::getToeTipsPoint(const std::vector<cv::Point2f> &masks,const float start_min_height_y,obs_joint_t &toe_tips_point,obs_joint_t &heel_point){

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