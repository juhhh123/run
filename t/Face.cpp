//
// Created by dell on 2024/6/3.
//

#include "Face.h"
#include <sstream>
#include <string>
#include "Logger.h"
#include "Flags.h"
#include "json.h"

namespace obs {

    static inline float dotProduct(const std::vector<float> &array1, const std::vector<float> &array2) {
        float result = 0.0f;
        for (int i = 0; i < array1.size(); ++i) {
            result += array1[i] * array2[i];
        }
        return result;
    }

    static inline float norm(const std::vector<float> &array) {
        float result = 0.0f;
        for (int i = 0; i < array.size(); ++i) {
            result += std::pow(array[i], 2);
        }
        return std::sqrt(result);
    }

    //计算矩阵余弦距离
    static inline float cosineSimilarity(const std::vector<float> &array1, const std::vector<float> &array2) {

        float dotProd = dotProduct(array1, array2);
        float norm1 = norm(array1);
        float norm2 = norm(array2);
        if (norm1 < EPISON || norm2 < EPISON)
            return 0.0f;
        else
            return std::fabs(dotProd / (norm1 * norm2));
    }

    Face::Face() {

    }

    Face::~Face() {
        UnInit();
    }

    bool Face::Init(const char *config_path){

        obs_config_path_ = std::string(config_path);
        std::string db_path = obs_config_path_ + "/database/";
        if (access(db_path.c_str(), F_OK) != 0){
            return false;
        }
        db_path += std::string(DATABASE_NAME);

        obsConnectionPool = new OBSConnectionPool(db_path.c_str(),CONNECTION_NUM);

        sqlite3 *db =  obsConnectionPool->pullConnection();
        if (db == nullptr) return false;

        char* errMsg;
        int rc = sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, &errMsg);
        if (rc != SQLITE_OK) {
            sqlite3_free(errMsg);
            obsConnectionPool->returnConnection(db);
            return false;
        }

        std::ostringstream sqlStream;
        sqlStream << "CREATE TABLE IF NOT EXISTS " << TABLE_NAME << " ("
                  << COLUMN_ID << " INTEGER PRIMARY KEY AUTOINCREMENT, "
                  << COLUMN_USER_ID << " TEXT UNIQUE, "
                  << COLUMN_USER_NAME << " TEXT NOT NULL, "
                  << COLUMN_FEATURES << " BLOB"
                  << ");";
        const std::string sql = sqlStream.str();

        if (sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &errMsg) != SQLITE_OK) {
            sqlite3_free(errMsg);
            sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
            obsConnectionPool->returnConnection(db);
            return false;
        }

        rc = sqlite3_exec(db, "COMMIT;", nullptr, nullptr, &errMsg);
        if (rc != SQLITE_OK) {
            sqlite3_free(errMsg);
            obsConnectionPool->returnConnection(db);
            return false;
        }

        obsConnectionPool->returnConnection(db);

        return true;
    }

    void Face::UnInit(){
        if (obsConnectionPool){
            delete obsConnectionPool;
            obsConnectionPool = nullptr;
        }
    }

    bool Face::UserIsExist(sqlite3 *db, const char *uid){

        bool exists = false;
        sqlite3_stmt* stmt = nullptr;

        std::ostringstream sqlStream;
        sqlStream << "SELECT "
                  << COLUMN_ID
                  << " FROM " << TABLE_NAME
                  << " WHERE user_id = ?;";
        const std::string sql = sqlStream.str();

        if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
            return false;
        }

        sqlite3_bind_text(stmt, 1, uid, -1, SQLITE_STATIC);

        if (sqlite3_step(stmt) == SQLITE_ROW) {
            exists = true;
        }

        sqlite3_finalize(stmt);

        return exists;
    }

    bool Face::Register(const char *userId,const char *userName,const std::vector<float>& features){

        std::lock_guard<std::mutex> lock(sqlMutex_);
        if (userId == NULL || userName == NULL || features.empty()){
            return false;
        }

        sqlite3 *db =  obsConnectionPool->pullConnection();
        if (db == nullptr) return false;

        if (UserIsExist(db,userId)){
            obsConnectionPool->returnConnection(db);
            return false;
        }

        char* errorMessage = nullptr;
        if (sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, &errorMessage) != SQLITE_OK) {
            sqlite3_free(errorMessage);
            obsConnectionPool->returnConnection(db);
            return false;
        }

        std::ostringstream sqlStream;
        sqlStream << "INSERT INTO " << TABLE_NAME << " ("
                  << COLUMN_USER_ID << ", "
                  << COLUMN_USER_NAME << ", "
                  << COLUMN_FEATURES << ") VALUES (?, ?, ?);";

        const std::string sql = sqlStream.str();

        sqlite3_stmt* stmt = nullptr;

        if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
            sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, &errorMessage);
            obsConnectionPool->returnConnection(db);
            return false;
        }

        sqlite3_bind_text(stmt, 1, userId, -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 2, userName, -1, SQLITE_STATIC);
        size_t featureSize = features.size() * sizeof(float);
        sqlite3_bind_blob(stmt, 3, features.data(), featureSize, SQLITE_STATIC);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
            sqlite3_finalize(stmt);
            sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, &errorMessage);
            obsConnectionPool->returnConnection(db);
            return false;
        }

        sqlite3_finalize(stmt);

        bool isSuccess = false;

        if (sqlite3_exec(db, "COMMIT;", nullptr, nullptr, &errorMessage) != SQLITE_OK) {
            sqlite3_free(errorMessage);
        }else{
            isSuccess = true;
        }

        obsConnectionPool->returnConnection(db);

        return isSuccess;
    }

    bool Face::Update(const char *userId, const char *userName, const std::vector<float>& features) {
        std::lock_guard<std::mutex> lock(sqlMutex_);
        if (userId == nullptr || userName == nullptr || features.empty()) {
            return false;
        }

        sqlite3 *db = obsConnectionPool->pullConnection();
        if (db == nullptr) return false;

        if (!UserIsExist(db, userId)) {
            obsConnectionPool->returnConnection(db);
            return false;
        }

        char* errorMessage = nullptr;
        if (sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, &errorMessage) != SQLITE_OK) {
            sqlite3_free(errorMessage);
            obsConnectionPool->returnConnection(db);
            return false;
        }

        std::ostringstream sqlStream;
        sqlStream << "UPDATE " << TABLE_NAME
                  << " SET " << COLUMN_USER_NAME << " = ?, "
                  << COLUMN_FEATURES << " = ? "
                  << "WHERE " << COLUMN_USER_ID << " = ?;";

        const std::string sql = sqlStream.str();

        sqlite3_stmt* stmt = nullptr;
        if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
            sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, &errorMessage);
            obsConnectionPool->returnConnection(db);
            return false;
        }

        sqlite3_bind_text(stmt, 1, userName, -1, SQLITE_STATIC);
        size_t featureSize = features.size() * sizeof(float);
        sqlite3_bind_blob(stmt, 2, features.data(), featureSize, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 3, userId, -1, SQLITE_STATIC);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
            sqlite3_finalize(stmt);
            sqlite3_exec(db,"ROLLBACK;",nullptr,nullptr,&errorMessage);
            obsConnectionPool->returnConnection(db);
            return false;
        }

        sqlite3_finalize(stmt);

        bool isSuccess = false;
        if (sqlite3_exec(db,"COMMIT;",nullptr,nullptr,&errorMessage) != SQLITE_OK){
            sqlite3_free(errorMessage);
        } else{
            isSuccess = true;
        }

        obsConnectionPool->returnConnection(db);

        return isSuccess;
    }

    bool Face::Delete(const char *userId) {

        std::lock_guard<std::mutex> lock(sqlMutex_);
        if (userId == nullptr) {
            return false;
        }

        sqlite3 *db = obsConnectionPool->pullConnection();
        if (db == nullptr) return false;

//        // 检查用户是否存在
//        if (!UserIsExist(db, userId)) {
//            obsConnectionPool->returnConnection(db);
//            return false;
//        }

        char* errorMessage = nullptr;
        if (sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, &errorMessage) != SQLITE_OK) {
            sqlite3_free(errorMessage);
            obsConnectionPool->returnConnection(db);
            return false;
        }

        std::ostringstream sqlStream;
        sqlStream << "DELETE FROM " << TABLE_NAME
                  << " WHERE " << COLUMN_USER_ID << " = ?;";

        const std::string sql = sqlStream.str();

        sqlite3_stmt* stmt = nullptr;
        if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
            sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, &errorMessage);
            obsConnectionPool->returnConnection(db);
            return false;
        }
        sqlite3_bind_text(stmt, 1, userId, -1, SQLITE_STATIC);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
            sqlite3_finalize(stmt);
            sqlite3_exec(db,"ROLLBACK;",nullptr,nullptr,&errorMessage);
            obsConnectionPool->returnConnection(db);
            return false;
        }

        sqlite3_finalize(stmt);

        bool isSuccess = false;
        if (sqlite3_exec(db,"COMMIT;",nullptr,nullptr,&errorMessage) != SQLITE_OK){
            sqlite3_free(errorMessage);
        } else{
            isSuccess = true;
        }

        obsConnectionPool->returnConnection(db);

        return isSuccess;
    }

    bool Face::RecognitionFace(const std::vector<float>& features, std::vector<obs_user_t>& users) {

        std::lock_guard<std::mutex> lock(sqlMutex_);
        if (features.empty()) {
            return false;
        }

        sqlite3 *db = obsConnectionPool->pullConnection();
        if (db == nullptr) return false;

        std::ostringstream sqlStream;
        sqlStream << "SELECT "
                  << COLUMN_USER_ID << ", "
                  << COLUMN_USER_NAME << ", "
                  << COLUMN_FEATURES
                  << " FROM " << TABLE_NAME << ";";
        const std::string sql = sqlStream.str();

        sqlite3_stmt* stmt = nullptr;
        if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
            obsConnectionPool->returnConnection(db);
            return false;
        }

        while (sqlite3_step(stmt) == SQLITE_ROW) {

            int blobSize = sqlite3_column_bytes(stmt, 2);
            if (blobSize % sizeof(float) != 0 || blobSize <= 0) {//是float的整数倍
                continue;
            }
            std::vector<float> featuresBlob(blobSize / sizeof(float));
            const void* blobData = sqlite3_column_blob(stmt, 2);
            memcpy(featuresBlob.data(), blobData, blobSize);
            float score = cosineSimilarity(featuresBlob,features);
            if (score > FACE_SCORE) {
                const char* userId = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
                const char* userName = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
                obs_user_t user;
                strncpy(user.userId, userId, sizeof(user.userId) - 1);
                user.userId[sizeof(user.userId) - 1] = '\0';
                strncpy(user.userName, userName, sizeof(user.userName) - 1);
                user.userName[sizeof(user.userName) - 1] = '\0';
                user.score = score;
                users.push_back(user);
                LOGE("userId:%s,userName:%s",user.userId,user.userName);
                break;
            }
        }

        sqlite3_finalize(stmt);
        obsConnectionPool->returnConnection(db);

        return !users.empty();
    }

}
