#pragma once

namespace lvins {

/**
 * @brief 禁止拷贝类，继承此类的类均禁止拷贝
 */
class NonCopyable {
public:
    NonCopyable(const NonCopyable &)            = delete;
    NonCopyable &operator=(const NonCopyable &) = delete;

protected:
    NonCopyable()  = default;
    ~NonCopyable() = default;
};

} // namespace lvins
