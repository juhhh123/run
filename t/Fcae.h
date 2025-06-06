//
// Created by dell on 2024/6/3.
//

#ifndef IROAD_PHYSICAL_SPORTS_APK_DEMO_FACE_H
#define IROAD_PHYSICAL_SPORTS_APK_DEMO_FACE_H
#include <iostream>
#include <vector>
#include <mutex>
#include "opencv2/opencv.hpp"
#include "CommonStruct.h"
#include "OBSConnectionPool.h"

using namespace std;
using namespace cv;

#define DATABASE_NAME "face.db"
#define TABLE_NAME "user"
#define COLUMN_ID "id"
#define COLUMN_USER_ID "user_id"
#define COLUMN_USER_NAME "user_name"
#define COLUMN_FEATURES "features"
#define CONNECTION_NUM 3

namespace obs {

    class Face {

    public:
        Face();

        virtual ~Face();

        bool Init(const char *config_path);

        void UnInit();

        bool Register(const char *userId,const char *userName,const std::vector<float>& features);

        bool Update(const char *userId,const char *userName,const std::vector<float>& features);

        bool Delete(const char * userId);

        bool RecognitionFace(const std::vector<float>& features,std::vector<obs_user_t> &users);

    protected:

        bool UserIsExist(sqlite3 *db,const char *uid);

    private:
        Face(const Face&);
        const Face& operator = (const Face&);

        std::string obs_config_path_;
        OBSConnectionPool *obsConnectionPool = nullptr;
        std::mutex sqlMutex_;

    };
}

#endif //RKNN_YOLOV5_ANDROID_APK_DEMO_FACE_H
