//
// Created by dell on 2024/4/19.
//

#include "LocationPoint.h"


namespace obs {

    LocationPoint::LocationPoint(){

    }

    LocationPoint::~LocationPoint(){

    }

    void LocationPoint::clear()
    {
        location = 0;
        b_first = false;
        enter_in_ = false;
        out_range_count_ = 0;
        localRect.clear();
        startTime_ = 0;
        timeOut_ = 0;
        recog_rate_ = 0;
    }

    void LocationPoint::reset()
    {
        b_first = false;
        enter_in_ = false;
        out_range_count_ = 0;
        startTime_ = 0;
        timeOut_ = 0;
        recog_rate_ = 0;
    }

    void LocationPoint::startTimer(uint64_t timeOut){
        startTime_ = TimesUtil::GetLocalMillTimeStamp();
        timeOut_ = timeOut;
    }

    bool LocationPoint::isTimeOut(){
        if(startTime_ == 0){
            return false;
        }

        if(TimesUtil::GetLocalMillTimeStamp() - startTime_ >= timeOut_){
            return true;
        }

        return false;
    }

}
