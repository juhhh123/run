//
// Created by dell on 2024/3/15.
//

#ifndef IROAD_PHYSICAL_SPORTS_COMMONSTRUCT_H
#define IROAD_PHYSICAL_SPORTS_COMMONSTRUCT_H

#include <iostream>
#include <vector>
#include "DataType.h"
#include <android/log.h>
#include <error.h>
#include <errno.h>

#define ALLOW_IMAGE_UNDISTORTPOINTS 0

#define OBS_MAX_PATH_LENGTH 256
#define OBS_COURSE_NAME_TXT_SIZE 80
#define OBS_SIP_ID_SIZE 25
#define RECV_BUFFER_SIZE 40960
#define M_PI       3.14159265358979323846
#define FRONT_STATE 0
#define LEFT_SIDE 1
#define RIGHT_SIDE 2
#define OBS_THREAD_NUM 1
#define OBS_FACE_THREAD_NUM 3
#define OBS_SKIP_MAX 2
#define OBS_PULLUP_MAX 4
#define OBS_ARMHANG_MAX 4
#define EPISON 1e-6
#define DIFFER 0.1
#define MARK_SCORE 0.70
#define MAX_CELL_SUM 5
#define OBS_ELBOW_SHOW_ANGLE 90
#define FACE_SCORE 0.55
#define OUT_RANGE_COUNT 20
#define RECOGNITION_RATE 5
#define READY_OUT_RANGE_COUNT 4

#define OBJ_NAME_MAX_SIZE 64
#define OBJ_NUMB_MAX_SIZE 128

#define RUNNING_RECOGNITION_FACE_RATE 4

#define PROTO_HEIGHT 160
#define PROTO_WEIGHT 160
#define USER_ID_SIZE 20
#define NAME_STR_SIZE 30
#define DIFFEN_WIDTH 1.2
#define FACE_RECT_SIZE 112
#define BUFF_STR_SIZE 256
#define DIFFEN_FACE_MIN_HEIGHT 0.75f
#define BOX_MIN_AREA 600.0f

//骨架写入本地文件
#define WRITE_BODIES_DATA 0
//读取本地骨架文件测试
#define LOCATION_FILE_TEST 0


#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "rknn3588", ##__VA_ARGS__);
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, "rknn3588", ##__VA_ARGS__);
#define LOG_ERROR(...) __android_log_print(ANDROID_LOG_ERROR, "rknn3588", ##__VA_ARGS__);
#define LOG_FATAL(...) __android_log_print(ANDROID_LOG_FATAL, "rknn3588", ##__VA_ARGS__);

using namespace iroad;

namespace obs {

    typedef enum
    {
        //无错误
        SPORT_NONE_ERROR = 0,

        //左臂未抬起
        JUMP_PING_LEFT_ARM_NOT_UP = 1,

        //左臂未抬起
        JUMP_PING_RIGHT_ARM_NOT_UP = 2,

        //双臂未抬起
        JUMP_PING_TWO_ARMS_NOT_UP = 3,

        //左臂抬起不够高
        JUMP_PING_LEFT_ARM_NOT_UP_ENOUGH = 4,

        //右臂抬起不够高
        JUMP_PING_RIGHT_ARM_NOT_UP_ENOUGH = 5,

        //双臂抬起不够高
        JUMP_PING_TWO_ARMS_NOT_UP_ENOUGH = 6,

        //双臂未夹紧
        JUMP_PING_TWO_ARMS_UMCLAMP = 7,

        //两腿未张开
        JUMP_PING_TWO_LEGS_NOT_OPEN = 8,

        //两腿张开距离不够
        JUMP_PING_TWO_LEGS_NOT_OPEN_ENOUGH = 9,

        //单腿平移，未起跳
        JUMP_PING_AN_LEG_MOVE_WITHOUT_ANOTHER = 10,

        //脚掌未动，双膝张开
        JUMP_PING_FEET_NOT_OPEN_BUT_KNEES_OPEN = 11,

        //两腿未并拢
        JUMP_PING_TWO_LEGS_UMCLAMP = 12,

        //提腿地面距离不足
        HIGH_LEG_NOT_ENOUGH = 13,

        //交替提腿
        HIGH_LEG_NOT_ALTERNAT_LEG_LIFT = 14,

        //未抱头
        SIT_UP_NOT_HUG_HRAD = 15,

        //未触膝
        SIT_UP_NOT_TOUCH_KNEE = 16,

        //未躺平
        SIT_UP_NOT_LIE = 17,

        //跳远踩线
        LONG_JUMP_TOUCH_LINE = 18,

        //检测区有多个人
        LONG_JUMP_MORE_PEOPLE = 19,

        //单脚起跳
        LONG_JUMP_SINGLE_LEG = 20,

        //跑步踩线
        RUNNING_JUMP_TOUCH_LINE = 21,

        //抢跑
        RUNNING_JUMP_THE_GUN = 22,

        //跳远出界（跳出界）
        LONG_JUMP_OUT_AREA = 23,

        //引体向上拉起时下颚未过杆
        PULL_UP_JAW_NOT_OVER_ROD = 24,

        //引体向上下落时手臂未伸直
        PULL_UP_ARM_NOT_STRAIGHTENING = 25,

        //引体向上脚触杆-结束
        PULL_UP_FEET_TOUCH_ROD = 26,

        //手离杆-结束
        PULL_UP_HAND_LEAVE_ROD = 27,

        //曲臂悬垂手离杆-结束
        ARM_HANG_LEAVE_ROD = 28,

        //曲臂悬垂脚触杆-结束
        ARM_HANG_FEET_TOUCH_ROD = 29,

        //仰卧起做站起身-结束
        SIT_UP_STAND_UP = 30,

        //仰卧起坐躺地超过10秒-结束
        SIT_UP_LIE_OVER_10S = 31,

        //双杠臂屈伸脚触杆-结束
        BAR_DIPS_FEET_TOUCH_ROD = 32,

        //双杠臂屈伸手离杆-结束
        BAR_DIPS_HAND_LEAVE_ROD = 33,

        //俯卧撑站起身-结束
        PUSH_UP_STAND_UP = 34,

        //俯卧撑身体触碰地面-结束
        PUSH_UP_BODY_TOUCH_GROUND = 35,

        //俯卧撑撑地手臂未伸直
        PUSH_UP_HAND_NOT_EXTENDED = 36,

        //俯卧撑下伏双肩关节高于肘关节
        PUSH_UP_HAND_NOT_BENT_ENOUGH = 37,

        //俯卧撑身体未平直
        PUSH_UP_BODY_NOT_EXTENDED = 38,

        //双杠臂屈伸下伏时肩关节高于肘关节
        BAR_DIPS_HAND_NOT_BENT_ENOUGH = 39,

        //双杠臂屈伸起身时手臂未伸直
        BAR_DIPS_HAND_NOT_EXTENDED = 40,

        //跳远已跳出，但起跳时踩线
        LONG_JUMP_TOUCH_START_LINE_AND_OUT = 41,

        //双杠支撑前移手离杆-结束
        BAR_WALK_LEAVE_ROD = 42,

        //跳绳中断
        SKIP_ROPE_INTERRUPT = 43,

        //颠臀
        SIT_UP_NEW_SWAYING_HIP = 44,

        //膝盖未弯曲
        SIT_UP_NEW_KNEE_NOT_BEND = 45,

        //手未过止点线
        SIT_UP_NEW_FINGER_NOT_TOUCH_ENDLINE = 46,

        //坐位体前屈膝盖弯曲
        SIT_AND_REACH_KNEE_BEND = 47,

        //坐位体前屈加速冲刺
        SIT_AND_REACH_ACCELERATE_FORWARD = 48,

        //坐位体前屈单手前驱
        SIT_AND_REACH_ONE_HAND_FORWARD = 49,

        //新版本仰卧起坐站起身-结束
        SIT_UP_NEW_STAND_UP = 50,

        //无绳空跳
        SKIP_ROPE_WITHOUT_ROPE = 51,

        //跳远冲刺跳
        LONG_JUMP_SPRINT_JUMP = 52,

        //跳远界外跳
        LONG_JUMP_OUTSIDE_JUMP = 53,

        //跳远接力跳
        LONG_JUMP_RELAY_JUMP = 54,

        //跳远单脚跳
        LONG_JUMP_SINGLE_FOOT = 55,

        //跳远距离过近，无成绩
        LONG_JUMP_NO_GRADE = 56,

        //起跳时站立在起跳线右边
        LONG_JUMP_OVER_START_LINE = 57,

        //未绕杆
        ROUND_POLE_FAIL = 58

    } obs_sports_error_type_t;

    typedef enum
    {
        //踩线检测
        CHECK_TOUCH_LINE = 0,

        //抢跑检测
        CHECK_JUNP_GUN = 1,

        //撞线检测
        CHECK_ENDING = 2,

        //ready检测(返回人脸框，举手准备)
        CHECK_READY = 3,

    }obs_running_check_status_t;

    typedef enum
    {
        //not ready
        NOT_READY = 0,

        //ready
        READY_STATUS = 1,

        //撞线
        END_STATUS = 2,

        //起跑
        START_STATUS = 3,

        //绕杆第一次过线
        END_STATUS_1 = 4,

        //绕杆第二次过线
        END_STATUS_2 = 5,

        //绕杆
        ROUND_POLE_OK = 6

    }obs_running_status_t;

    typedef struct {

        int cameraIndex  = 0;
        int id;
        char userId[USER_ID_SIZE] = {0};
        float sucess_count = 0.0f;
        int fail_count = 0.0f;
        /*
         * default: -1
         * 0: enter of circle
         * 1: out of circle
         * */
        int status = -1;
        /*
         * default: -1
         * 0: not interrupt
         * 1: interrupt
         * */
        int interrupt = -1;
        float timeKeeping = 0.0f;
        float distance = 0.0f;
        int fail_id = 0;
        int64_t msec = 0;

        /**
        * 0： not ready
        * 1：ready
        * 2：end
        * 3: start
        */
        int over_flag = 0;

        cv::Rect_<float> rect;
        obs_joint_t joints[OBS_JOINT_MAX];

        //举手状态
        //1:左手
        //2：右手
        int raise_hand = 0;

        //人脸框
        cv::Rect_<float> faceRect;

    } obs_person_t;

    typedef struct {

        int cameraIndex  = 0;
        int location = 0;
        /*
         * default: -1
         * 0: 无人
         * 1: 有人
         * */
        int exist_person = -1;
        int raise_hand = 0;
        int fail_id = 0;

        cv::Rect_<float> faceRect;

    }obs_location_t;

    struct obs_line_t{

        int index = 0;
        obs_joint_t a;
        obs_joint_t b;

        obs_line_t(){};

        obs_line_t(obs_joint_t a, obs_joint_t b){
            this->a = a;
            this->b = b;
        }
        obs_line_t(int _x1, int _y1, int _x2, int _y2){
            this->a.x = _x1;
            this->a.y = _y1;
            this->b.x = _x2;
            this->b.y = _y2;
        }
    };

    typedef struct {
        //波动幅度
        float min_distance_times = 0.0f;
        //手腕最高最低差值
        float low0_low1_times = 0.0f;
        //停止帧率
        float fps = 0.0f;
        //停止帧率倍数
        float max_fps_times = 0.0f;
        //身体起步阶段幅度
        float pre_min_distance_times = 0.0f;
        //倒减波动幅度
        float differ_min_distance_times = 0.0f;

    } obs_skip_rope_config;

    typedef struct {
        float _candy_para1 = 0.0f;
        float _candy_para2 = 0.0f;
        float _candy_para3 = 0.0f;

        float _haugh_para1 = 0.0f;
        float _haugh_para2 = 0.0f;
        float _haugh_para3 = 0.0f;
        float _haugh_para4 = 0.0f;
    } obs_long_jump_config;

    typedef struct {
        float up_threshold = 0.0f;
        float elbow_angle = 0.0f;
    }obs_pull_up_config;

    typedef struct {
        char userId[USER_ID_SIZE] = {0};
        char userName[NAME_STR_SIZE] = {0};
        float score = 0.0f;
    }obs_user_t;

    struct Line{
        Point2f point1;
        Point2f point2;
        Line(Point2f _x1, Point2f _y1){
            point1 = _x1;
            point2 = _y1;
        }
        Line(int _x1, int _y1, int _x2, int _y2){
            point1.x = _x1;
            point1.y = _y1;
            point2.x = _x2;
            point2.y = _y2;
        }
    };

    struct FaceNode {
        char uid[USER_ID_SIZE] = {0};
        char name[NAME_STR_SIZE] = { 0 };
        std::vector<float> features;
        FaceNode* next = nullptr;

        // 构造函数
        FaceNode(const char* uid_, const char* name_, const std::vector<float>& features_, FaceNode* next_ = nullptr)
                :next(next_)
        {
            strncpy(uid, uid_, USER_ID_SIZE);
            strncpy(name, name_, NAME_STR_SIZE);
            features = features_;
        }
    };

    struct obs_sit_up_new_line_t{
        float x1 = 0.0f;
        float y1 = 0.0f;
        float x2 = 0.0f;
        float y2 = 0.0f;

        obs_sit_up_new_line_t(){}

        obs_sit_up_new_line_t(const float x1_, const float y1_,const float x2_, const float y2_ ){
            this->x1 = x1_;
            this->y1 = y1_;
            this->x2 = x2_;
            this->y2 = y2_;
        }
        obs_sit_up_new_line_t(const Point2f p1, const Point2f p2){
            this->x1 = p1.x;
            this->y1 = p1.y;
            this->x2 = p2.x;
            this->y2 = p2.y;
        }
        obs_sit_up_new_line_t(const obs_joint_t joint1, const obs_joint_t joint2){
            this->x1 = joint1.x;
            this->y1 = joint1.y;
            this->x2 = joint2.x;
            this->y2 = joint2.y;
        }
    };

}

#endif
