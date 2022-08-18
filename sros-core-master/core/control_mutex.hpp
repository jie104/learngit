/**
 * @file control_mutex
 *
 * @author pengjiali
 * @date 20-1-9.
 *
 * @describe sros控制权的锁
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_CONTROL_MUTEX_H
#define SROS_CONTROL_MUTEX_H

namespace sros {
namespace core {

class ControlMutex {
 public:
    class Locker {
     public:
        void reset() { *this = Locker(); }

        uint64_t session_id = 0;  // 当前单独控制agv用户的session_id
        std::string user_name;    // 有时候用用户名区分
        std::string ip_address;   // 当前单独控制agv用户的ip
        std::string nick_name;    // 当前单独控制agv用户的绰号
    };

    /**
     * 获取sros控制权，当锁被别人独占了，就无法后去控制权，当获取控制权失败就不允许执行控制命令
     * @param session_id
     * @return
     */
    bool get(uint64_t session_id) const {
        boost::shared_lock<boost::shared_mutex> lock(mutex);
        return !is_locked || session_id == locker.session_id;
    }

    /**
     * 获取当前独占的人
     * @return
     */
    Locker getLocker() const {
        boost::shared_lock<boost::shared_mutex> lock(mutex);
        return locker;
    }

    uint64_t getLockerSessionId() const {
        boost::shared_lock<boost::shared_mutex> lock(mutex);
        return locker.session_id;
    }

    /**
     * 查询当前是否被独占了
     * @return
     */
    bool isLock() const {
        boost::shared_lock<boost::shared_mutex> lock(mutex);
        return is_locked;
    }

    /**
     * 获取当前是否被用户为session_id的用户独占
     * @param session_id
     * @return
     */
    bool isLock(uint64_t session_id) const {
        boost::shared_lock<boost::shared_mutex> lock(mutex);
        return is_locked && session_id == locker.session_id;
    }

    /**
     * 获取当前是否被用户名为 user_name 的用户独占
     * @param user_name
     * @return
     */
    bool isLock(const std::string &user_name) const {
        boost::shared_lock<boost::shared_mutex> lock(mutex);
        return is_locked && user_name == locker.user_name;
    }

    /**
     * 独占
     * @param session_id
     * @param ip_address
     * @param user_name
     * @param nick_name
     * @return
     */
    bool lock(uint64_t session_id, const std::string &ip_address, const std::string &user_name,
              const std::string &nick_name) {  // 独占
        boost::lock_guard<boost::shared_mutex> lock(mutex);

        if (is_locked) return false;

        is_locked = true;
        locker.session_id = session_id;
        locker.ip_address = ip_address;
        locker.user_name = user_name;
        locker.nick_name = nick_name;
        return true;
    }

    /**
     * 解除独占
     */
    void unlock() {  // 解除独占
        boost::lock_guard<boost::shared_mutex> lock(mutex);

        is_locked = false;
        locker.reset();
    }

 private:
    mutable boost::shared_mutex mutex;
    bool is_locked = false;  // 独占锁是否被锁上
    Locker locker;
};

}  // namespace core
}  // namespace sros
#endif  // SROS_CONTROL_MUTEX_H
