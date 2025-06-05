//
// Created by Lige on 2024/4/27.
//

#include "Squatting.h"
#include <map>
#include "json.h"
#include "Logger.h"
#include "Flags.h"

namespace obs {

    Squatting::Squatting(const std::string config_path){
        LOGI("Create squatting object");
        app_root_path_ = std::string(config_path);
        int nPos = app_root_path_.rfind('/');
        app_root_path_ = app_root_path_.substr(0,nPos);

        Json::Reader json_reader;
        Json::Value js;
        ifstream infile(config_path.c_str(), ios::binary);
        if(!infile.is_open())
        {
            LOGE("Open {%s} failed!",config_path.c_str());
            return;
        }

        if(!json_reader.parse(infile, js)) {
            LOGE("parse {%s} failed!",config_path.c_str());
            infile.close();
            return;
        }
        infile.close();

        Json::Value locationList = js["LocationList"];
        for (int i = 0; i < locationList.size(); ++i) {
            int min_x = 10000;
            int min_y = 10000;
            int max_x = 0;
            int max_y = 0;
            int index = locationList[i]["index"].asInt();
            std::shared_ptr<SquattingCount> SquattObj(new SquattingCount());
            SquattObj->location_ = index;
            Json::Value rectPointList = locationList[i]["RectPoint"];
            for (int i = 0; i < rectPointList.size(); ++i) {
                int x = rectPointList[i]["x"].asInt();
                int y = rectPointList[i]["y"].asInt();
                if (x < min_x) {
                    min_x = x;
                }
                if (y < min_y) {
                    min_y = y;
                }
                if (x > max_x){
                    max_x = x;
                }
                if (y > max_y){
                    max_y = y;
                }
                SquattObj->locationPoint.push_back(obs_joint_t(x,y));
            }
            SquattObj->rect_.x = min_x;
            SquattObj->rect_.y = min_y;
            SquattObj->rect_.width = max_x - min_x;
            SquattObj->rect_.height = max_y - min_y;

            SquattingSubMap_[index] = SquattObj;
        }
    }

    Squatting::~Squatting(){

    }

    void Squatting::Execute(const int cameraIndex,const cv::Mat &frame,const int location,const int64_t pts,std::vector<obs_person_t> &persons){

        if(poseForward_ == nullptr || frame.empty() || SquattingSubMap_.find(location) == SquattingSubMap_.end()){
            return;
        }
        std::vector<obs_body_t> bodies;
        poseForward_(frame,location,bodies);
        if(bodies.empty()){
            LOGE("location:%d body empty",location);
        }

        if (location == 1 || location == 3){
            std::shared_ptr<SquattingCount> obj = SquattingSubMap_[location];
            obs_person_t person;
            person.id = obj->location_;
            bool exist_person = false;
            std::map<float, obs_body_t> bodyMap;
            for (int i = 0; i < bodies.size(); ++i) {
                const obs_body_t &body = bodies[i];
                obs_joint_t left_ankle = body.joints[OBS_JOINT_ANKLE_LEFT];
                obs_joint_t right_ankle = body.joints[OBS_JOINT_ANKLE_RIGHT];
                if(Flags::isPointInPolygon(obj->locationPoint,left_ankle) ||
                   Flags::isPointInPolygon(obj->locationPoint,right_ankle)){

                    //计算相交面积
                    float area = Flags::computeIntersectionArea(body.rect,obj->rect_);
                    if(area > EPISON){
                        bodyMap[area] = body;
                    }
                }
            }
            //获取最大面积的body
            auto bodyIter = bodyMap.rbegin();
            if (bodyIter != bodyMap.rend()) {
                const obs_body_t &body = bodyIter->second;
                obj->CountSquatting(obj->location_, body);

                if(obj->fail_id_) {
                    person.msec = pts;
                    person.fail_id = obj->fail_id_;
                }

                if(person.msec)
                    LOGE("error-sec:%ld",person.msec);

                person.interrupt = obj->interrupt_status_;
                person.status = 0;
                exist_person = true;
                obj->out_range_count_ = 0;
                person.rect = body.rect;
                memcpy(person.joints,body.joints,sizeof(obs_joint_t) * OBS_JOINT_MAX);
            }

            //out of range
            if(!exist_person){
                if(obj->out_range_count_ ++ > OUT_RANGE_COUNT){
                    person.status = 1;
                    obj->uid_ = 0;
                }
            }

            person.fail_count = obj->fail_count_;
            person.sucess_count =  obj->count_number_;
            persons.push_back(person);

        }else{
            for (auto iter = SquattingSubMap_.begin(); iter != SquattingSubMap_.end();iter++) {

                std::shared_ptr<SquattingCount> obj = iter->second;
                if (obj->location_ == 3 || obj->location_ == 1){
                    continue;
                }
                obs_person_t person;
                person.id = obj->location_;

                bool exist_person = false;
                std::map<float, obs_body_t> bodyMap;
                for (int i = 0; i < bodies.size(); ++i) {
                    const obs_body_t &body = bodies[i];
                    obs_joint_t left_ankle = body.joints[OBS_JOINT_ANKLE_LEFT];
                    obs_joint_t right_ankle = body.joints[OBS_JOINT_ANKLE_RIGHT];
                    if(Flags::isPointInPolygon(obj->locationPoint,left_ankle) ||
                       Flags::isPointInPolygon(obj->locationPoint,right_ankle)){

                        //计算相交面积
                        float area = Flags::computeIntersectionArea(body.rect,obj->rect_);
                        if(area > EPISON){
                            bodyMap[area] = body;
                        }
                    }
                }

                //获取最大面积的body
                auto bodyIter = bodyMap.rbegin();
                if (bodyIter != bodyMap.rend()) {
                    const obs_body_t &body = bodyIter->second;
                    obj->CountSquatting(obj->location_, body);

                    if(obj->fail_id_) {
                        person.msec = pts;
                        person.fail_id = obj->fail_id_;
                    }

                    if(person.msec)
                        LOGE("error-sec:%ld",person.msec);

                    person.interrupt = obj->interrupt_status_;
                    person.status = 0;
                    exist_person = true;
                    obj->out_range_count_ = 0;
                    person.rect = body.rect;
                    memcpy(person.joints,body.joints,sizeof(obs_joint_t) * OBS_JOINT_MAX);
                }

                //out of range
                if(!exist_person){
                    if(obj->out_range_count_ ++ > OUT_RANGE_COUNT){
                        person.status = 1;
                        obj->uid_ = 0;
                    }
                }

                person.fail_count = obj->fail_count_;
                person.sucess_count =  obj->count_number_;
                persons.push_back(person);
            }
        }
    }

}
