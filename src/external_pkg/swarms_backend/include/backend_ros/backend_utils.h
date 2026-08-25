//
// Created by nhy on 2023/11/10.
//

//
// Created by ljy on 23-2-19.
//

/**
 * json 字符串中解析c++基础类型数据
 */

#ifndef MQTT_NODE_UTILS_H
#define MQTT_NODE_UTILS_H

#include <math.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <regex>
#include "eigen3/Eigen/Eigen"

#define RAD_TO_DEG 57.29577951308232 //180.0/3.141592653589793238463

namespace math_utils {

    // 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // q0 q1 q2 q3
    // w x y z
    Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q) {
        float quat[4];
        quat[0] = q.w();
        quat[1] = q.x();
        quat[2] = q.y();
        quat[3] = q.z();

        Eigen::Vector3d ans;
        ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                       1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
        ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
        ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]),
                       1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
        return ans;
    }

    // 从(roll,pitch,yaw)创建四元数  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
    Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy) {
        // YPR - ZYX
        return Eigen::Quaterniond(
                Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
        );
    }


}

class JsonUtils {
public:
    static float format(float f, int digits = 2) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(digits) << f; //保留3位小数
        return std::stof(ss.str());
    }

    static void processStateMap(std::map<std::string, int> &stateMap, std::map<int, std::string> &stateIndexMap) {
        for (auto it = stateMap.begin(); it != stateMap.end(); ++it) {
            // 获取键值对
            std::string stateName = it->first;
            int stateValue = it->second;
            stateIndexMap.insert(std::make_pair(stateValue, stateName));
        }
    }

    static void parseMissionYaml(const std::string file_path, std::map<std::string, int> &missionType) {
        // 加载YAML文件
        YAML::Node missionConfig = YAML::LoadFile(file_path + "/config/mission_type.yaml");
        // 遍历YAML文件并输出其内容
        for (YAML::const_iterator it = missionConfig.begin(); it != missionConfig.end(); ++it) {
            // 获取键值对
            std::string missionName = it->first.as<std::string>();
            int missionId = it->second.as<int>();
            missionType.insert(std::make_pair(missionName, missionId));
        }
    }

    static void parseYaml(const std::string file_path, std::map<std::string, int> &state) {
        // 加载YAML文件
        YAML::Node algoConfig = YAML::LoadFile(file_path + "/config/algo_id.yaml");
        YAML::Node algoIds = algoConfig["AlgoID"].as<YAML::Node>();
        // 遍历YAML文件并输出其内容
        for (YAML::const_iterator it = algoIds.begin(); it != algoIds.end(); ++it) {
            // 获取键值对
            std::string algoName = it->first.as<std::string>();
            int subStateId = it->second.as<int>();
            state.insert(std::make_pair(algoName, subStateId));
        }
    }

    static void parseMsgFile(const std::string file_path, std::map<std::string, int> &state) {

        // 打开文件并读取所有内容
        std::ifstream file(file_path + "/msg/UsmMajorState.msg");
        std::string line;
        if (!file.is_open()) {
            std::cerr << "Error: Failed to open file: " << file_path << std::endl;
        }
        // 在每一行中查找匹配的字符串
        while (std::getline(file, line)) {

            // 去掉空白行和注释
            if (line.empty() || line[0] == '#') {
                continue;
            }

            // 查找等号的位置
            size_t pos = line.find('=');
            if (pos == std::string::npos) {
                continue;
            }
            // 按照等号分割两部分
            std::string name_str = JsonUtils::trim(line.substr(0, pos));
            std::string key_str = JsonUtils::trim(line.substr(pos + 1));

            //// 按照空格分割name_str为多个部分
            std::istringstream iss(name_str);
            std::vector<std::string> name_parts;
            std::string name_part;
            while (iss >> name_part) {
                name_parts.push_back(name_part);
            }
            // 将字符串转为整数
            int majorStateId = std::stoi(key_str);

            std::string name = name_parts[1];
            size_t index = name.find_last_of("_");
            if (index == std::string::npos) {
                state.insert(std::make_pair(name, majorStateId));
            } else {
                std::string algoName = JsonUtils::trim(name.substr(index + 1));
                state.insert(std::make_pair(algoName, majorStateId));
            }

        }
    }

    static std::string trim(const std::string &str) {
        std::regex re(R"(^\s+|\s+$)");  // 匹配开头或结尾的空白字符
        return std::regex_replace(str, re, "");  // 使用空字符串替换匹配到的空白字符
    }

/**
*
* @param msg exception message
* @param file the file of  current exception
* @param line the line of  current exception
* @param func the function of  current exception
* @return
*/

    static std::string getExceptionMsg(const std::string msg, const std::string &file, int line,
                                       const std::string &func) {
        std::stringstream ss;
        ss << "File: " << file << ", Line: " << line << ", Function: " << func << " - ";
        ss << "Message: " << msg;
        return ss.str();
    }

    ~JsonUtils() {};
};


#endif //MQTT_NODE_UTILS_H