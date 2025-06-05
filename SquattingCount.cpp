//
// Created by Lige on 2024/4/27.
//

#include "SquattingCount.h"

namespace obs{

    static inline int obs_sum(std::list<int> num_list) {
        int sum = 0;
        for (std::list<int>::iterator it = num_list.begin(); it != num_list.end(); ++it) {
            sum += *it;
        }
        return sum;
    }

    SquattingCount::SquattingCount(){
        m_list_leftShoulder.clear();
        m_list_leftHip.clear();
        m_UpTimes = 0.0f;
        m_DownTimes = 0.0f;
    }

    SquattingCount::~SquattingCount(){

    }

    int SquattingCount::CountSquatting(int index, const obs_body_t& body){

//        if (index != 2)
//            return 0;
        m_lefthipY          = body.joints[OBS_JOINT_HIP_LEFT].y;
        m_leftshoulderY     = body.joints[OBS_JOINT_SHOULDER_LEFT].y;
        m_rightshoulderY    = body.joints[OBS_JOINT_SHOULDER_RIGHT].y;
        m_righthipY         = body.joints[OBS_JOINT_HIP_RIGHT].y;
        m_leftAnkleY        = body.joints[OBS_JOINT_ANKLE_LEFT].y;
        m_rightAnkleY       = body.joints[OBS_JOINT_ANKLE_RIGHT].y;
        if(m_leftAnkleY == 0.0){
            return 0;
        }

        m_maxDistanceLeftShouklder_Ankle = m_maxDistanceLeftShouklder_Ankle > fabs(m_leftshoulderY - m_leftAnkleY) ?
                                           m_maxDistanceLeftShouklder_Ankle: fabs(m_leftshoulderY - m_leftAnkleY);

        //m_maxDistanceRightShouklder_Ankle
        m_maxDistanceLeftHip_Ankle = m_maxDistanceLeftHip_Ankle > fabs(m_leftAnkleY - m_lefthipY)?
                                     m_maxDistanceLeftHip_Ankle : fabs(m_leftAnkleY - m_lefthipY);

        m_leftshoulderY = m_lefthipY;
        m_rightshoulderY = m_righthipY;
        m_maxDistanceLeftShouklder_Ankle = m_maxDistanceLeftHip_Ankle;


//        m_list_leftShoulder.push_back(m_leftshoulderY);
//        m_list_rightShoulder.push_back(m_rightshoulderY);
//
//        if(m_list_leftShoulder.size() == 4){
//            m_list_leftShoulder.pop_front();
//            m_list_rightShoulder.pop_front();
//        }
//        if(m_list_leftShoulder.size() < 3){
//            return 0;
//        }
//        std::list<float>::iterator itl = m_list_leftShoulder.begin();
//        std::list<float>::iterator itr = m_list_rightShoulder.begin();
//        float yl0 = *itl,yr0 = *itr;
//        ++itl;
//        ++itr;
//        float yl1 = *itl,yr1 = *itr;
//        ++itl;
//        ++itr;
//        float yl2 = *itl,yr2 = *itr;
//
//        if(yl0 < yl1 && yl1 < yl2){
//            isHighest = false;
//            isLowest = false;
//        }
//
//        if(yl0 > yl1 && yl1 > yl2){
//            isHighest = false;
//            isLowest = false;
//        }
//
//        //上
//        if(yl1 < yl0 && yl1 <= yl2){
//            isHighest = true;
//            isLowest = false;
//            m_highestShoulder = yl1;
//        }
//
//        //最低点
//        if(yl1 > yl0 && yl1 >= yl2){
//            isHighest = false;
//            isLowest = true;
//            m_lowestShoulder = yl1;
//        }
//        LOGI("count_number_:%f, m_UpTimes:%f, m_DownTimes:%f, distance_Shoulder_hip:%f, maxdistance:%f.\n", count_number_, m_UpTimes, m_DownTimes,
//             fabs(m_leftshoulderY - m_leftAnkleY), m_maxDistanceLeftShouklder_Ankle);

//        if(isLowest && !isHighest){
            if(fabs(m_leftshoulderY - m_leftAnkleY)  < m_maxDistanceLeftShouklder_Ankle * 0.65
            && fabs(m_rightshoulderY - m_rightAnkleY)  < m_maxDistanceLeftShouklder_Ankle * 0.65
            && isUpest){
                LOGI("down\n");
                m_DownTimes += 0.5f;
                isUpest = false;
            }
//        }

//        if(!isLowest && isHighest){
            if(fabs(m_leftshoulderY - m_leftAnkleY)  > m_maxDistanceLeftShouklder_Ankle * 0.65
            && fabs(m_rightshoulderY - m_rightAnkleY)  > m_maxDistanceLeftShouklder_Ankle * 0.65
               && !isUpest){
                LOGI("up\n");
                m_UpTimes += 0.5f;
                isUpest = true;
            }
//        }

        if(fabs(m_UpTimes - m_DownTimes) >= 1){
            if(m_UpTimes > m_DownTimes)
                m_UpTimes -= 0.5f;
            else
                m_DownTimes -= 0.5f;
        }


        count_number_ = m_DownTimes + m_UpTimes;
        count_number_ = floor(count_number_ + 0.1);


        ////////////////////////////////////////////////////
        if(count_number_list.size() > 0 && count_number_ == *(count_number_list.rbegin())){
            if(fabs(count_number_) > EPISON){
                count_no_list++;
                if(this->count_no_list >= std::max(1.2*average_value,50.0)){

                    exit_ = 1;
                    exit_triggered = 0;
                }
            }
        }else{
            count_number_list.push_back(count_number_);
            num_list.push_back(count_no_list);
            if(num_list.size() >= 4){
                num_list.pop_front();
            }
            average_value = obs_sum(num_list) / num_list.size();
            count_no_list = 0;
            if(exit_ == 1 && fabs(count_number_) > EPISON){
                if(exit_triggered == 1){
                    exit_ = 0;
                }else{
                    exit_triggered = 1;
                }
            }
            if(exit_ == 1 && count_number_ > 1.0f && exit_sub == 1){
                count_number_ = count_number_ - 0.5f;
                count_number_list.pop_front();
                exit_sub = 0;
            }else{
                exit_sub = 1;
            }
        }

        interrupt_status_ = exit_;

        return exit_;

    }
}