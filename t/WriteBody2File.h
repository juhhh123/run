//
// Created by pine-nut on 2024/4/19.
//

#ifndef RKNN_YOLOV5_ANDROID_APK_DEMO_WRITEBODY2FILE_H
#define RKNN_YOLOV5_ANDROID_APK_DEMO_WRITEBODY2FILE_H

#include <iostream>
#include <fstream>
#include "CommonStruct.h"

class WriteBody2File {
public:
    WriteBody2File(std::string file_name);
    ~WriteBody2File();

    bool write(const std::vector<obs_body_t> &bodies);
    void readyToRead();
    bool read(std::vector<obs_body_t> &bodies);
    void close();
protected:
    FILE *file_;
    long long size_;    // 总共的 bodies 数量
    long long read_size_;
};


#endif //RKNN_YOLOV5_ANDROID_APK_DEMO_WRITEBODY2FILE_H
