#ifndef CONFIG_H
#define CONFIG_H
#include "myslam/common_include.h"
namespace myslam
{
//单例模式，只有一个全局对象。
class Config
{
private:
    static std::shared_ptr<Config>config_;
    cv::FileStorage file_;
    Config(){}
public:
    ~Config();
    // 只能通过这个函数构造，构造的对象是一个智能指针，可以自己析构
    static void setParameterFile(const std::string& filename);

    // 使用模板，返回任意类型的字段
    template <typename T>
    static T get(const std::string& key)
    {
        return T(Config::config_->file_[key]);
    }
};

}
#endif