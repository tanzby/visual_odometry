#include "config.h"

std::shared_ptr<Config> Config::config_ = nullptr;

bool Config::LoadConfig(const std::string& config_path)
{
    if (config_ == nullptr)
        config_ = std::shared_ptr<Config>(new Config);

    try
    {
        config_->config_root_ = YAML::LoadFile(config_path);
        return true;
    }
    catch (std::exception& e)
    {
        LOG(ERROR) << "cannot load config file from" << config_path;
        return false;
    }
}

Config::Config(){}


