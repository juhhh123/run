//
// Created by dell on 2024/3/19.
//

#ifndef IROAD_PHYSICAL_SPORTS_FLAGS_H
#define IROAD_PHYSICAL_SPORTS_FLAGS_H
#include <iostream>
#include <random>
#include <cmath>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include <fstream>
#include "libyuv.h"
#include "opencv2/opencv.hpp"
#include "json.h"
#include "CommonStruct.h"
#include "DataType.h"


#define ZQ_MAX(a, b) ((a) > (b) ? (a) : (b))
#define ZQ_MIN(a, b) ((a) < (b) ? (a) : (b))

using namespace cv;
using namespace std;
using namespace libyuv;

namespace obs {
    extern int g_camera_direction_;
    class Flags {
    public:
        static inline std::string RandomString(const int len)
        {
            std::string charSet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
            std::string randomString = "";
            std::random_device rd;
            std::default_random_engine e(rd());
            std::uniform_int_distribution<unsigned> u(0, charSet.length());
            srand((int)time(0));
            for (int i = 0; i < len; i++) {
                int randomPoz = u(e) % charSet.length();
                randomString += charSet.substr(randomPoz, 1);
            }
            return randomString;
        }

        static inline void YUV420pToBGR(const uint8_t *src,const int width,const int height,cv::Mat &out){
            if(src == NULL){
                return;
            }

            const uint8_t* src_y = src;
            const uint8_t* src_u = src + width * height;
            const uint8_t* src_v = src + width * height * 5/4;

            out.create(height, width, CV_8UC3);
            libyuv::I420ToRGB24(
                    src_y, width,
                    src_u, (width+1) >> 1,
                    src_v, (width+1) >> 1,
                    out.data, width * 3,
                    width, height);
        }

        // 将 std::vector<Line> 转换为 Json::Value
        static Json::Value vectorToJson(const std::vector<Line> &lines) {
            Json::Value linesJson(Json::arrayValue);
            for (const auto &line : lines) {
                linesJson.append(lineToJson(line));
            }
            Json::Value result;
            result["LinePointList"] = linesJson;
            return result;
        }

        // 将 Line 转换为 Json::Value
        static Json::Value lineToJson(const Line &line) {
            Json::Value lineJson;
            lineJson["x1"] = line.point1.x;
            lineJson["x2"] = line.point2.x;
            lineJson["y1"] = line.point1.y;
            lineJson["y2"] = line.point2.y;
            return lineJson;
        }

        static inline void YV12ToBGR(const uint8_t *src,const int width,const int height,cv::Mat &out){
            if(src == NULL){
                return;
            }
            const uint8_t* src_y = src;
            const uint8_t* src_v = src + width * height;
            const uint8_t* src_u = src + width * height * 5/4;

            out.create(height, width, CV_8UC3);
            libyuv::I420ToRGB24(
                    src_y, width,
                    src_u, (width+1) >> 1,
                    src_v, (width+1) >> 1,
                    out.data, width * 3,
                    width, height);
        }


        static inline void YUVJ420pToBGR(const uint8_t *src,const int width,const int height,cv::Mat &out){
            if(src == NULL){
                return;
            }
            const uint8_t* src_y = src;
            const uint8_t* src_u = src + width * height;
            const uint8_t* src_v = src + width * height * 5/4;

            out.create(height, width, CV_8UC3);
            libyuv::J420ToRGB24(
                    src_y, width,
                    src_u, (width+1) >> 1,
                    src_v, (width+1) >> 1,
                    out.data, width * 3,
                    width, height);
        }

        static inline void NV12ToBGR(const uint8_t *src,const int width,const int height,cv::Mat &out){

            if(src == NULL){
                return;
            }

            out.create(height, width, CV_8UC3);
            libyuv::NV12ToRGB24(
                    src, width,
                    src + width * height, width,
                    out.data,width*3,
                    width, height);

        }

        static inline void NV21ToBGR(const uint8_t *src,const int width,const int height, cv::Mat &out){

            if(src == NULL){
                return;
            }

            out.create(height, width, CV_8UC3);
            libyuv::NV21ToRGB24(
                    src, width,
                    src + width * height, width,
                    out.data,width*3,
                    width, height);

        }

        static inline void NV21ToNV12Mirror(const uint8_t *src,const int width,const int height, uint8_t * out){

            if(src == NULL){
                return;
            }
            uint8_t *des_nv12 = new uint8_t[width * height * 3/2];
            libyuv::NV21ToNV12(
                    src, width,
                    src + width * height, width,
                    des_nv12,width,
                    des_nv12 + width * height,width,
                    width, height
            );

            libyuv::NV12Mirror(
                    des_nv12,width,
                    des_nv12 + width * height, width,
                    out,width,
                    out + width * height, width,
                    width, height
            );

            delete [] des_nv12;
            des_nv12 = NULL;
        }

        static inline void BGRToYUV420p(cv::Mat &bgr,uint8_t* out_yuv420_data){
            const int src_width = bgr.cols;
            const int src_height = bgr.rows;
            int src_y_size = src_width * src_height;
            int src_u_size = (src_width >> 1) * (src_height >> 1);
            int rgb24_stride = 3 * src_width;

            uint8_t* src_i420_y_data = out_yuv420_data;
            uint8_t* src_i420_u_data = out_yuv420_data + src_y_size;
            uint8_t* src_i420_v_data = out_yuv420_data + src_y_size + src_u_size;

            uint8_t *bgr_data = bgr.ptr<uint8_t>();

            libyuv::RGB24ToI420(bgr_data, rgb24_stride,
                                src_i420_y_data, src_width,
                                src_i420_u_data, src_width >> 1,
                                src_i420_v_data, src_width >> 1,
                                src_width, src_height);
        }

        static inline void BGRToYUVJ420p(cv::Mat &bgr,uint8_t* out_yuv420_data){
            const int src_width = bgr.cols;
            const int src_height = bgr.rows;
            int src_y_size = src_width * src_height;
            int src_u_size = (src_width >> 1) * (src_height >> 1);
            int rgb24_stride = 3 * src_width;

            uint8_t* src_i420_y_data = out_yuv420_data;
            uint8_t* src_i420_u_data = out_yuv420_data + src_y_size;
            uint8_t* src_i420_v_data = out_yuv420_data + src_y_size + src_u_size;

            uint8_t *bgr_data = bgr.ptr<uint8_t>();

            libyuv::RGB24ToJ420(bgr_data, rgb24_stride,
                                src_i420_y_data, src_width,
                                src_i420_u_data, src_width >> 1,
                                src_i420_v_data, src_width >> 1,
                                src_width, src_height);
        }

        static inline void YUVCrop(const uint8_t *src,const int width,const int height,const cv::Rect checkRect,uint8_t *out){
            if (src == NULL || out == NULL) {
                return;
            }

            const uint8_t* src_y = src;
            const uint8_t* src_v = src + width * height;
            const uint8_t* src_u = src + width * height * 5 / 4;

            int crop_width = checkRect.width;
            int crop_height = checkRect.height;

            for (int row = 0; row < crop_height; ++row) {
                const uint8_t* src_row = src_y + (checkRect.y + row) * width + checkRect.x;
                uint8_t* dst_row = out + row * crop_width;
                memcpy(dst_row, src_row, crop_width);
            }

            int uv_width = width / 2;
            int crop_uv_width = crop_width / 2;
            int crop_uv_height = crop_height / 2;

            for (int row = 0; row < crop_uv_height; ++row) {
                const uint8_t* src_row_v = src_v + (checkRect.y / 2 + row) * uv_width + checkRect.x / 2;
                uint8_t* dst_row_v = out + crop_width * crop_height + row * crop_uv_width;
                memcpy(dst_row_v, src_row_v, crop_uv_width);
                const uint8_t* src_row_u = src_u + (checkRect.y / 2 + row) * uv_width + checkRect.x / 2;
                uint8_t* dst_row_u = out + crop_width * crop_height + crop_uv_height * crop_uv_width + row * crop_uv_width;
                memcpy(dst_row_u, src_row_u, crop_uv_width);
            }
        }

        static inline void YUVCropNew(const uint8_t *src, const int width, const int height, const cv::Rect checkRect, std::shared_ptr<uint8_t> out) {
            if (src == nullptr || out == nullptr) {
                return;
            }

            const uint8_t* src_y = src;
            const uint8_t* src_v = src + width * height;
            const uint8_t* src_u = src + width * height * 5 / 4;

            int crop_width = checkRect.width;
            int crop_height = checkRect.height;

            for (int row = 0; row < crop_height; ++row) {
                const uint8_t* src_row = src_y + (checkRect.y + row) * width + checkRect.x;
                uint8_t* dst_row = out.get() + row * crop_width; // 使用 .get() 获取原始指针
                memcpy(dst_row, src_row, crop_width);
            }

            int uv_width = width / 2;
            int crop_uv_width = crop_width / 2;
            int crop_uv_height = crop_height / 2;

            for (int row = 0; row < crop_uv_height; ++row) {
                const uint8_t* src_row_v = src_v + (checkRect.y / 2 + row) * uv_width + checkRect.x / 2;
                uint8_t* dst_row_v = out.get() + crop_width * crop_height + row * crop_uv_width;
                memcpy(dst_row_v, src_row_v, crop_uv_width);

                const uint8_t* src_row_u = src_u + (checkRect.y / 2 + row) * uv_width + checkRect.x / 2;
                uint8_t* dst_row_u = out.get() +
                                     crop_width * crop_height +
                                     crop_uv_height * crop_uv_width +
                                     row * crop_uv_width;
                memcpy(dst_row_u, src_row_u, crop_uv_width);
            }
        }

        //适合小于100的整数
        static inline string DecimalToHex(int decimal)
        {
            string hex = "";
            int num = decimal;
            while (decimal > 0)
            {
                int remainder = decimal % 16;
                if (remainder < 10)
                    hex = (char)(remainder + '0') + hex;
                else
                    hex = (char)(remainder + 'A' - 10) + hex;
                decimal /= 16;
            }

            if (num < 16) {
                hex = "0" + hex;
            }
            return hex;
        }

        static inline std::string to_string_txt(float val,int fixed)
        {
            std::ostringstream out;
            out << std::fixed << std::setprecision(fixed) << val;
            return out.str();
        }


        static inline float findAngle(obs_joint_t joint_1, obs_joint_t joint_2, obs_joint_t joint_3)
        {
            float diffy3y2=joint_3.y-joint_2.y;
            float diffy2y1=joint_1.y-joint_2.y;
            float diffx3x2=joint_3.x-joint_2.x;
            float diffx2x1=joint_1.x-joint_2.x;

            //[-pi,pi]
            float angle=(std::atan2(diffy3y2,diffx3x2)-std::atan2(diffy2y1,diffx2x1))*180/M_PI;

            if (angle < 0)
            {
                angle += 360;
                if (angle > 180) {
                    angle = 360 - angle;
                }
            }else if(angle > 180)
            {
                angle = 360 - angle;
            }

            return angle;

        }

        static inline void interp(int N_x, int N_xp, const float x, const float* xp, const float* yp, float& y, float left, float right)
        {
            y = left;
            int ip = 0;
            int ip_next = 1;
            int i = 0;
            while (i < N_x) {
                float m = (yp[ip_next] - yp[ip]) / (xp[ip_next] - xp[ip]);
                float q = yp[ip] - m * xp[ip];
                //std::cout << m << " "<< q<<std::endl;
                while (x< xp[ip_next]) {
                    if (x>= xp[ip])
                        y = m * x + q;
                    //std::cout << y[i] << std::endl; // debug: OK
                    i += 1;
                    if (i >= N_x) { break; }
                    //std::cout << i << " " <<ip <<std::endl;
                }
                ip += 1;
                ip_next += 1;
                if (ip_next == N_xp) {
                    while (i < N_x) {
                        y = right;
                        i++;
                    }
                    break;
                }
            }
        }

        static inline int sit_up_judege_side(const obs_body_t &body)
        {
            int side_flag=FRONT_STATE;
            obs_joint_t left_ear=body.joints[OBS_JOINT_EAR_LEFT];
            obs_joint_t right_ear=body.joints[OBS_JOINT_EAR_RIGHT];

            obs_joint_t left_shoulder=body.joints[OBS_JOINT_SHOULDER_LEFT];
            obs_joint_t right_shoulder=body.joints[OBS_JOINT_SHOULDER_RIGHT];

            obs_joint_t left_hip=body.joints[OBS_JOINT_HIP_LEFT];
            obs_joint_t right_hip=body.joints[OBS_JOINT_HIP_RIGHT];

            obs_joint_t left_knee=body.joints[OBS_JOINT_KNEE_LEFT];
            obs_joint_t right_knee=body.joints[OBS_JOINT_KNEE_RIGHT];

            obs_joint_t left_ankle=body.joints[OBS_JOINT_ANKLE_LEFT];
            obs_joint_t right_ankle=body.joints[OBS_JOINT_ANKLE_RIGHT];
            //判断左侧
            if (left_ear.score > right_ear.score && left_knee.score > right_knee.score)
            {
                if (left_hip.x > left_ankle.x && left_knee.x > left_ankle.x) {
                    side_flag = LEFT_SIDE;
                }
            }
            else if(right_ear.score > left_ear.score && right_knee.score > left_knee.score){
                if (right_hip.x < right_ankle.x && right_knee.x < right_ankle.x) {
                    side_flag = RIGHT_SIDE;
                }
            }

            return side_flag;
        }

        static inline int push_up_judege_side(obs_body_t body) {
            int side_flag = FRONT_STATE;
            obs_joint_t left_ear = body.joints[OBS_JOINT_EAR_LEFT];
            obs_joint_t right_ear = body.joints[OBS_JOINT_EAR_RIGHT];

            obs_joint_t left_shoulder = body.joints[OBS_JOINT_SHOULDER_LEFT];
            obs_joint_t right_shoulder = body.joints[OBS_JOINT_SHOULDER_RIGHT];

            obs_joint_t left_hip = body.joints[OBS_JOINT_HIP_LEFT];
            obs_joint_t right_hip = body.joints[OBS_JOINT_HIP_RIGHT];

            obs_joint_t left_knee = body.joints[OBS_JOINT_KNEE_LEFT];
            obs_joint_t right_knee = body.joints[OBS_JOINT_KNEE_RIGHT];

            obs_joint_t left_ankle = body.joints[OBS_JOINT_ANKLE_LEFT];
            obs_joint_t right_ankle = body.joints[OBS_JOINT_ANKLE_RIGHT];

            if (left_ear.score > right_ear.score && left_knee.score > right_knee.score)
            {
                if (left_hip.x < left_ankle.x && left_knee.x < left_ankle.x) {
                    side_flag = LEFT_SIDE;
                }

            }
            else if (right_ear.score > left_ear.score && right_knee.score > left_knee.score) {

                if (right_ankle.x < right_hip.x && right_ankle.x < right_knee.x) {
                    side_flag = RIGHT_SIDE;
                }
            }

            return side_flag;
        }

        static inline bool is_point_valid(obs_joint_t p)
        {
            int pos_x = static_cast<int>(p.x);
            int pos_y = static_cast<int>(p.x);

            bool valid = true;
            if (pos_x <= 0 && pos_y <= 0)
            {
                valid = false;
            }

            return valid;
        }

        static inline void stringSplit(std::string str, const char split, std::vector<std::string>& res)
        {
            std::istringstream iss(str);	// 输入流
            std::string token;			// 接收缓冲区
            while (std::getline(iss, token, split))	// 以split为分隔符
            {
                res.push_back(token);
            }
        }

        static inline double get_us(struct timeval t) {
            return (t.tv_sec * 1000000 + t.tv_usec);
        }


        static inline void UndistortPoints(const std::vector<cv::Point2f> &src,std::vector<cv::Point2f> &out){

            if(src.empty()){
                return;
            }

            const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 866.2796418025142, 0, 661.1614839090564,
                    0, 868.8708942808281, 326.550691694077,
                    0, 0, 1);

            const cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.4249895396252572,
                    0.2338508034479069,
                    0.003084345446289188,
                    -0.0002781395820446112,
                    -0.07374533902190321);

            cv::undistortPoints(src, out, cameraMatrix, distCoeffs,cv::noArray(), cameraMatrix);
        }

        static inline float crossProduct(const obs_joint_t &p1, const obs_joint_t &p2, const obs_joint_t &p3) {
            return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
        }

        static inline bool distanceToeTipsCal(const obs_line_t &line1,const obs_line_t &line2){

            float c1 = crossProduct(line1.a, line1.b, line2.a);
            float c2 = crossProduct(line1.a, line1.b, line2.b);
            float c3 = crossProduct(line2.a, line2.b, line1.a);
            float c4 = crossProduct(line2.a, line2.b, line1.b);

            if ((c1 * c2 < 0) && (c3 * c4 < 0)) {
                return true;
            }

            return false;
        }

        static inline int checkPointPosition(const obs_joint_t &p, const obs_joint_t &p1, const obs_joint_t &p2,bool UpAndDown = true){
            if(UpAndDown) {
                if (p1.x == p2.x) {
                    //垂直线
                    return -2;
                }
                // 计算直线的斜率
                float slope = (p2.y - p1.y) / (p2.x - p1.x);//-1

                // 计算点到直线的垂直距离
                float distance = (p.y - p1.y) - slope * (p.x - p1.x);

                if (distance > 0.0f)
                    return 1;  // 下方
                else if (distance < 0.0f)
                    return -1; // 上方
                else
                    return 0;  // 在直线上
            }else{
                double result = (p2.x - p1.x) * (p.y - p1.y) - (p2.y - p1.y) * (p.x - p1.x);
                if (result > 0) {
                    //直线左边
                    return 1;
                } else if (result < 0) {
                    //直线右边
                    return -1;
                } else {
                    //直线上
                    return 0;
                }
            }
        }

        static inline int checkPointPositionLine(const obs_joint_t &point,const obs_line_t &line){
            if(line.a.x == line.b.x){
                //垂直线
                return -2;
            }
            // 计算直线的斜率
            float slope = (line.b.y - line.a.y) / (line.b.x - line.a.x);

            // 计算点到直线的垂直距离
            float distance = (point.y - line.a.y) - slope * (point.x - line.a.x);

            if (distance > 0.0f)
                return 1;  // 下方
            else if (distance < 0.0f)
                return -1; // 上方
            else
                return 0;  // 在直线上
        }

        /**
         * 从给定的特征点计算仿射变换矩阵
         * @param lmk 特征点
         * @param image_size 输出的放射矩阵的size
         * @return 仿射变换矩阵
         */
        static inline cv::Mat estimate_norm(const cv::Mat& lmk, int image_size = 112) {
            assert(lmk.rows == 5 && lmk.cols == 2);
            assert(image_size % 112 == 0 || image_size % 128 == 0);

            const cv::Mat arcface_dst = (cv::Mat_<float>(5, 2) << 30, 46,
                    66, 46,
                    46, 66,
                    33, 86,
                    62, 86);

            float ratio = (image_size % 112 == 0) ? static_cast<float>(image_size) / 112.0f : static_cast<float>(image_size) / 128.0f;
            float diff_x = (image_size % 112 == 0) ? 0 : 8.0f * ratio;

            cv::Mat dst = arcface_dst.clone() * ratio;
            dst.col(0) += diff_x;

            cv::Mat tform = cv::estimateAffinePartial2D(lmk, dst);
            return tform;
        }

        /**
         *
         * @param src 使用计算出的仿射变换矩阵对输入图像（src）进行仿射变换。
         * @param out 输出的矩阵Mat
         * @param size 将变换后的图像存储在输出图像（out）中，并确保输出图像的尺寸为指定的size。
         * @param landmark 根据给定的特征点（landmark）计算仿射变换矩阵。
         */
        static inline bool ImgWarpAffine(const cv::Mat &src,cv::Mat &out,const cv::Size& size,const cv::Mat &landmark){

            cv::Mat M = estimate_norm(landmark, size.width);
            if (M.type() != CV_32F && M.type() != CV_64F){
                return false;
            }

            if (M.rows != 2 || M.cols != 3){
                return false;
            }

            cv::warpAffine(src, out, M,
                           size,
                           cv::INTER_LINEAR,
                           cv::BORDER_CONSTANT,
                           cv::Scalar(0, 0, 0));
            return true;
        }

        static inline bool isPointOnLineSegment(const obs_joint_t& a, const obs_joint_t& b, const obs_joint_t& p) {
            double crossProduct = (p.y - a.y) * (b.x - a.x) - (p.x - a.x) * (b.y - a.y);

            if (std::fabs(crossProduct) > 1e-9) {
                return false;
            }

            if ((p.x >= std::min(a.x, b.x) && p.x <= std::max(a.x, b.x)) &&
                (p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y))) {
                return true; // 在线段上
            }

            return false; // 不在范围内
        }

        //射线法
        static inline bool isPointInPolygon(const std::vector<obs_joint_t>& polygon, const obs_joint_t& point) {
            int n = polygon.size();
            bool inside = false;

            for (int i = 0, j = n - 1; i < n; j = i++) {

                if (isPointOnLineSegment(polygon[i], polygon[j], point)) {
                    return true; // 点在边上
                }

                if (((polygon[i].y > point.y) != (polygon[j].y > point.y)) &&
                    (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) /
                               (polygon[j].y - polygon[i].y) + polygon[i].x))
                {
                    inside = !inside;
                }
            }

            return inside;
        }

        static inline bool fileExists(const std::string& filename) {
            if (access(filename.c_str(), F_OK) != -1) {
                return true; // 文件存在
            } else {
                return false; // 文件不存在
            }
        }


        /**
         * 计算等分点的函数
         * @param p1 端点
         * @param p2 端点
         * @param n 等分数（如二等分则线段p1p2之间应该有一个点，所返回的点集为三个点）
         * @return 从p1到p2返回点集
         */
        static inline std::vector<Point> calculateEqualDivisions(const Point& p1, const Point& p2, int n) {
            std::vector<Point> points;
            points.push_back(p1);  // 将起点加入结果向量

            // 计算 x 和 y 方向上的增量
            double dx = (p2.x - p1.x) / n;
            double dy = (p2.y - p1.y) / n;

            // 计算每个等分点的坐标
            for (int i = 1; i < n; ++i) {
                Point p;
                p.x = p1.x + i * dx;
                p.y = p1.y + i * dy;
                points.push_back(p);
            }

            points.push_back(p2);  // 将终点加入结果向量
            return points;
        }

        // 计算等分点的函数
        static inline std::vector<obs_joint_t> calculateEqualDivisions(const obs_joint_t& p1, const obs_joint_t& p2, int n) {
            std::vector<obs_joint_t> points;
            points.push_back(p1);  // 将起点加入结果向量

            // 计算 x 和 y 方向上的增量
            double dx = (p2.x - p1.x) / n ;
            double dy = (p2.y - p1.y) / n ;

            // 计算每个等分点的坐标
            for (int i = 1; i < n; ++i) {
                obs_joint_t p;
                p.x = p1.x + i * dx;
                p.y = p1.y + i * dy;
                points.push_back(p);
            }

            points.push_back(p2);  // 将终点加入结果向量
            return points;
        }

        /**
         * 计算点到直线的距离
         * @param A 点A
         * @param B 直线的一个端点
         * @param C 直线的一个端点
         * @return 距离
         */
        static inline double pointToLineDistance(const cv::Point2f& A, const cv::Point2f& B, const cv::Point2f& C) {
            // 计算向量 d
            cv::Point2f d = C - B;

            // 计算向量 p
            cv::Point2f p = A - B;

            // 计算向量 n (d 的法向量)
            cv::Point2f n(-d.y, d.x);

            // 计算 n 的模 (长度)
            double n_length = std::sqrt(n.x * n.x + n.y * n.y);

            // 计算 n 和 p 的点积
            double dot_product = n.x * p.x + n.y * p.y;

            // 计算距离
            double distance = std::abs(dot_product) / n_length;

            return distance;
        }

        static inline double pointToLineDistance(const obs_joint_t& j1, const obs_joint_t& j2, const obs_joint_t& j3) {
            Point2f A(j1.x,j1.y);
            Point2f B(j2.x,j2.y);
            Point2f C(j3.x,j3.y);

            // 计算向量 d
            cv::Point2f d = C - B;

            // 计算向量 p
            cv::Point2f p = A - B;

            // 计算向量 n (d 的法向量)
            cv::Point2f n(-d.y, d.x);

            // 计算 n 的模 (长度)
            double n_length = std::sqrt(n.x * n.x + n.y * n.y);

            // 计算 n 和 p 的点积
            double dot_product = n.x * p.x + n.y * p.y;

            // 计算距离
            double distance = std::abs(dot_product) / n_length;

            return distance;
        }

        //相交面积
        static inline float calculateOverlapArea(const obs_joint_t &p1,const obs_joint_t &p2,const obs_joint_t &p3,const obs_joint_t &p4){

            if (p1.x > p4.x || p2.x < p3.x || p1.y > p4.y || p2.y < p3.y) {
                return 0; // 不相交，返回面积为0
            }

            // 计算相交部分的边界
            float overlapX1 = std::max(p1.x, p3.x);
            float overlapY1 = std::max(p1.y, p3.y);
            float overlapX2 = std::min(p2.x, p4.x);
            float overlapY2 = std::min(p2.y, p4.y);

            float width = overlapX2 - overlapX1;
            float height = overlapY2 - overlapY1;

            return width * height;
        }

        static inline float computeIntersectionArea(const cv::Rect_<float>& rect1, const cv::Rect_<float>& rect2) {
            cv::Rect intersection = rect1 & rect2;
            float area = intersection.area();
            return area;
        }

        /**
         *
         * @param line1 直线1
         * @param line2 直线2
         * @param intersection 直线1与直线2的交点
         * @return 成功或失败（不相交）
         */
        static inline bool getIntersectionPoint(const obs_line_t &line1, const obs_line_t &line2, obs_joint_t &intersection) {
            // 计算直线参数
            float A1 = line1.b.y - line1.a.y;
            float B1 = line1.a.x - line1.b.x;
            float C1 = A1 * line1.a.x + B1 * line1.a.y;

            float A2 = line2.b.y - line2.a.y;
            float B2 = line2.a.x - line2.b.x;
            float C2 = A2 * line2.a.x + B2 * line2.a.y;

            // 计算交点
            float denominator = A1 * B2 - A2 * B1;

            if (denominator == 0) {
                return false; // 平行或重合
            }

            intersection.x = (C1 * B2 - C2 * B1) / denominator;
            intersection.y = (A1 * C2 - A2 * C1) / denominator;

            return true;
        }

        // 判断点 P 是否在直线 AB 的左边、右边或在线上
        static int pointRelativeToLine(const Point2f& A, const Point2f& B, const Point2f& P) {
            float cross_product = (B.x - A.x) * (P.y - A.y) - (B.y - A.y) * (P.x - A.x);
            if (cross_product > 0) {
                return 1;  // 点 P 在直线 AB 的左边
            } else if (cross_product < 0) {
                return -1; // 点 P 在直线 AB 的右边
            } else {
                return 0;  // 点 P 在线上
            }
        }

        static inline void UndistortPoints(const obs_joint_t &src_joint, obs_joint_t &out_joint) {
            if (src_joint.x < EPISON || src_joint.y < EPISON) {
                return;
            }
            std::vector<cv::Point2f> src;
            std::vector<cv::Point2f> out;
            src.push_back(Point2f(src_joint.x, src_joint.y));
            Flags::UndistortPoints(src, out);
            if (out.empty()) {
                return;
            }
            out_joint.x = out.at(0).x;
            out_joint.y = out.at(0).y;
        }

        static inline void UndistortPoints(const obs_body_t &src_body, obs_body_t &out_body) {
            std::vector<cv::Point2f> src;
            std::vector<cv::Point2f> out;
            UndistortPoints(src_body.joints[OBS_JOINT_NOSE], out_body.joints[OBS_JOINT_NOSE]);
            UndistortPoints(src_body.joints[OBS_JOINT_EYE_LEFT],
                            out_body.joints[OBS_JOINT_EYE_LEFT]);
            UndistortPoints(src_body.joints[OBS_JOINT_EYE_RIGHT],
                            out_body.joints[OBS_JOINT_EYE_RIGHT]);
            UndistortPoints(src_body.joints[OBS_JOINT_EAR_LEFT],
                            out_body.joints[OBS_JOINT_EAR_LEFT]);
            UndistortPoints(src_body.joints[OBS_JOINT_EAR_RIGHT],
                            out_body.joints[OBS_JOINT_EAR_RIGHT]);
            UndistortPoints(src_body.joints[OBS_JOINT_SHOULDER_LEFT],
                            out_body.joints[OBS_JOINT_SHOULDER_LEFT]);
            UndistortPoints(src_body.joints[OBS_JOINT_SHOULDER_RIGHT],
                            out_body.joints[OBS_JOINT_SHOULDER_RIGHT]);
            UndistortPoints(src_body.joints[OBS_JOINT_ELBOW_LEFT],
                            out_body.joints[OBS_JOINT_ELBOW_LEFT]);
            UndistortPoints(src_body.joints[OBS_JOINT_RELBOW_RIGHT],
                            out_body.joints[OBS_JOINT_RELBOW_RIGHT]);
            UndistortPoints(src_body.joints[OBS_JOINT_WRIST_LEFT],
                            out_body.joints[OBS_JOINT_WRIST_LEFT]);
            UndistortPoints(src_body.joints[OBS_JOINT_WRIST_RIGHT],
                            out_body.joints[OBS_JOINT_WRIST_RIGHT]);
            UndistortPoints(src_body.joints[OBS_JOINT_HIP_LEFT],
                            out_body.joints[OBS_JOINT_HIP_LEFT]);
            UndistortPoints(src_body.joints[OBS_JOINT_HIP_RIGHT],
                            out_body.joints[OBS_JOINT_HIP_RIGHT]);
            UndistortPoints(src_body.joints[OBS_JOINT_KNEE_LEFT],
                            out_body.joints[OBS_JOINT_KNEE_LEFT]);
            UndistortPoints(src_body.joints[OBS_JOINT_KNEE_RIGHT],
                            out_body.joints[OBS_JOINT_KNEE_RIGHT]);
            UndistortPoints(src_body.joints[OBS_JOINT_ANKLE_LEFT],
                            out_body.joints[OBS_JOINT_ANKLE_LEFT]);
            UndistortPoints(src_body.joints[OBS_JOINT_ANKLE_RIGHT],
                            out_body.joints[OBS_JOINT_ANKLE_RIGHT]);
            UndistortPoints(src_body.joints[OBS_JOINT_NECK], out_body.joints[OBS_JOINT_NECK]);
        }

        static inline void UndistortPoints(const std::vector<obs_body_t> &src_body, std::vector<obs_body_t> &out_body) {
            std::vector<cv::Point2f> src;
            std::vector<cv::Point2f> out;
            obs_body_t tmp_body;
            for(int i=0;i<src_body.size();i++) {
                UndistortPoints(src_body[i],tmp_body);
                out_body.push_back(tmp_body);
            }
        }

        static inline void UndistortPoints(const std::vector<obs_mask_t>& src_mask,std::vector<obs_mask_t>& out_mask){
            if(src_mask.empty()){
                return;
            }
            if(!out_mask.empty()){
                out_mask.clear();
            }
            for(int i=0;i<src_mask.size();i++) {
                obs_mask_t mask_points;
                Flags::UndistortPoints(src_mask[i].masks, mask_points.masks);
                out_mask.push_back(mask_points);
            }
        }

    };
}

#endif //IROAD_PHYSICAL_SPORTS_FLAGS_H
