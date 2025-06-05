//
// Created by pine-nut on 2024/4/19.
//

#include "WriteBody2File.h"

#include "CommonStruct.h"

#include <string.h>


WriteBody2File::WriteBody2File(std::string file_name)
{
    file_ = fopen(file_name.c_str(), "ab+");
    if (file_ == NULL) {
        LOG_FATAL("Open file [%s] failed!!!, error: %s !!!", file_name.c_str(), strerror(errno));
        exit(0);
    }
    size_ = 0;
    read_size_ = 0;
}

WriteBody2File::~WriteBody2File()
{

}

bool WriteBody2File::write(const std::vector<obs_body_t> &bodies)
{
    if (file_ == NULL)
        return false;

    int num = bodies.size();
    fwrite(reinterpret_cast<char*>(&num), sizeof(num), 1, file_);
    int ret = fwrite(reinterpret_cast<const char*>(&bodies[0]), num*sizeof(bodies[0]) , 1, file_);
    LOGI("write file ret: %d", ret);
    if (ret != 1) {
        LOG_ERROR("ERROR: successfully written ret %d!!!\n", ret);
        return false;
    }
    fflush(file_);
    return true;
}

void WriteBody2File::readyToRead()
{
    if (file_ == NULL)
        return;

    // 移动到文件头
    fseek(file_, 0, SEEK_SET);
}

bool WriteBody2File::read(std::vector<obs_body_t> &bodies)
{
    if (file_ == NULL)
        return false;
    bodies.clear();
    int num;
    int ret = fread(reinterpret_cast<char*>(&num), sizeof(num), 1, file_);
    if (ret < 0) {
        LOG_ERROR("ERROR: successfully written ret %d !!!\n", ret);
        if (feof(file_)) {
            LOGI("Reached end of file\n");
        }
        return false;
    }
    bodies.resize(num);
    ret = fread(reinterpret_cast<char*>(&bodies[0]), num*sizeof(bodies[0]), 1, file_);
    LOGI("read file ret: %d, bodies.size:%ld\n", ret, bodies.size());
    if (ret != 1) {
        LOG_ERROR("ERROR: successfully written ret %d !!!\n", ret);
    }
    return true;
}

void WriteBody2File::close()
{
    if (file_ != NULL) {
        fclose(file_);
        file_ = NULL;
        LOGI("fclose...");
    }
}