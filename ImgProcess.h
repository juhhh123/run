//
// Created by Lige on 2024/6/17.
//

#ifndef GIT_YUEDONG_STATION_IMGPROCESS_H
#define GIT_YUEDONG_STATION_IMGPROCESS_H

#include "CommonStruct.h"
#include "json.h"

namespace obs {

    class ImgProcess {
    public:
        ImgProcess();
        ~ImgProcess();

        void InitImgParameter(const char *configPath,Json::Value jsobj);

        void img_handle(const uint8_t *frameData,cv::Mat &out);

        std::vector<Line> GetLongJumpLines(const cv::Mat& edges);

        std::vector<Point2f> _location;


        float _candy_para1 = 0.0f;
        float _candy_para2 = 0.0f;
        float _candy_para3 = 0.0f;

        float _haugh_para1 = 0.0f;
        float _haugh_para2 = 0.0f;
        float _haugh_para3 = 0.0f;
        float _haugh_para4 = 0.0f;

        std::vector<Point2f> _locationPoint_vec;

    private:


    };
}

#endif //GIT_YUEDONG_STATION_IMGPROCESS_H
