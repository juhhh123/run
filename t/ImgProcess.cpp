//
// Created by Lige on 2024/6/17.
//

#include "ImgProcess.h"
#include <fstream>
#include "Flags.h"


namespace obs{

    ImgProcess::ImgProcess(){

    }

    ImgProcess::~ImgProcess(){

    }

    void ImgProcess::InitImgParameter(const char *configPath,Json::Value jsobj){

        Json::Reader json_reader;
        Json::Value js;
        std::string config_path(configPath);
        std::ifstream infile(config_path.c_str(), std::ios::binary);
        if (!infile.is_open()) {
            LOGE("InitImgParameter Open {%s} failed!", config_path.c_str());
            return;
        }

        if (!json_reader.parse(infile, js)) {
            LOGE("InitImgParameter parse {%s} failed!", config_path.c_str());
            infile.close();
            return;
        }
        infile.close();

        _candy_para1 = js["Threshold"]["candy1"].asFloat() * 1.0f;
        _candy_para2 = js["Threshold"]["candy2"].asFloat() * 1.0f;
        _candy_para3 = js["Threshold"]["candy3"].asFloat() * 1.0f;
        _haugh_para1 = js["Threshold"]["haugh1"].asFloat() * 1.0f;
        _haugh_para2 = js["Threshold"]["haugh2"].asFloat() * 1.0f;
        _haugh_para3 = js["Threshold"]["haugh3"].asFloat() * 1.0f;
        _haugh_para4 = js["Threshold"]["haugh4"].asFloat() * 1.0f;

        Json::Value rectPoint = js["RectPoint"];
        for(int i=0;i<jsobj.size();i++){
            float x = rectPoint[i]["x"].asFloat();
            float y = rectPoint[i]["y"].asFloat();
            Point2f joint(x,y);
            _locationPoint_vec.push_back(joint);
        }
    }

    void ImgProcess::img_handle(const uint8_t *frameData,cv::Mat &out){

        if (!frameData) {
            return;
        }

        cv::Mat input_img;
        Flags::YV12ToBGR(frameData, 1280, 720, input_img);



//        float x = 1280.0f, y = 720.0f, w = 0.0f, h = 0.0f;
//        float max_x = 0.0f;
//        float max_y = 0.0f;
//        for(int i=0;i<_locationPoint_vec.size();i++){
//            if(_locationPoint_vec[i].x < x){
//                x = _locationPoint_vec[i].x;
//            }
//            if(_locationPoint_vec[i].y < y){
//                y = _locationPoint_vec[i].y;
//            }
//            if(_locationPoint_vec[i].x > max_x){
//                max_x = _locationPoint_vec[i].x;
//            }
//            if(_locationPoint_vec[i].y > max_y){
//                max_y = _locationPoint_vec[i].y;
//            }
//        }
//        w = max_x - x;
//        h = max_y - y;
//
//        Rect region_of_interest = Rect2f(x, y, w, h);
        // = input_img(region_of_interest);

        cv::Mat mask = cv::Mat::zeros(input_img.size(), CV_8UC1);

        // 填充多边形区域为白色
        std::vector<std::vector<cv::Point2f>> fillContAll;
        fillContAll.push_back(_locationPoint_vec);
        cv::fillPoly(mask, fillContAll, cv::Scalar(255));

        cv::Mat roi;
        input_img.copyTo(roi, mask);
        /*
         * 掩码 (mask) 是一张单通道图像，它与原图像 (image) 大小相同。掩码图像中的每个像素值决定
         * 了对应位置的像素是否会从原图像复制到目标图像。如果掩码中的像素值是非零值（例如，255 表示
         * 白色），则原图像中对应位置的像素会被复制。如果掩码中的像素值是零，则对应位置的像素会被忽略。
         * */


        // 灰度转换
        Mat gray;
        cvtColor(roi, gray, COLOR_BGR2GRAY);


        // 边缘检测
        Mat edges;
        Canny(gray, edges, _candy_para1, _candy_para2, _candy_para3);

    }

    std::vector<Line> ImgProcess::GetLongJumpLines(const cv::Mat& edges){
        std::vector<Vec4i> cv_lines;
        HoughLinesP(edges, cv_lines, 1, CV_PI / _haugh_para1, _haugh_para2, _haugh_para3, _haugh_para4);
        std::vector<Line> res;
        for (size_t i = 0; i < cv_lines.size(); i++) {
            Vec4i l;
            res.push_back(Line(static_cast<float>(l[0]),static_cast<float>(l[1]),
                               static_cast<float>(l[2]),static_cast<float>(l[3])));
        }
        return res;
    }


}