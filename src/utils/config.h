#pragma once

#include <yaml-cpp/yaml.h>
#include <vector>
#include <unordered_map>

// add support for eigen matrix
#include "eigen-yaml-serialization.h"

/**
 * @brief class for loading and get parameters, run as a Singleton
 *        wrapper of yaml-cpp
 */
class Config
{
    YAML::Node config_root_;

    static std::shared_ptr<Config> config_;

    Config();

public:

    Config(Config const&) = delete;

    Config& operator=(Config const&) = delete;

    static bool LoadConfig(const std::string& config_path);

    template <typename T>
    inline static T Get(const std::string& key, T default_value = T())
    {
        if (!config_->config_root_[key])
        {
            LOG(WARNING) << "cannot find '"<< key <<"' in loaded configuration file, using default value instead.";
            return default_value;
        }
        return config_->config_root_[key].as<T>();
    }
};