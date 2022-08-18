/**
 * @file pipe_module.h
 *
 * @author pengjiali
 * @date 18-12-13.
 *
 * @describe1、C++中管道不能用ifstream操作，ifstream能打开管道，但是Python那边打开管道就会自动崩溃，所以只能用C那一套读写文件
 * 2、C++open管道默认打开模式是阻塞方式，也就是到等待对方打开管道的另一端，open才会返回。
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef MODULES_PIPE_PIPE_MODULE_H_
#define MODULES_PIPE_PIPE_MODULE_H_

#include <fstream>
#include "core/module.h"

namespace sros {

class PipeModule : public sros::core::Module {
 public:
    PipeModule();
    virtual ~PipeModule() = default;

    void run() override;

 private:
    bool init();
    void doRead();

    void onUpgradeResult(sros::core::base_msg_ptr m);

    int read_fd_ = -1;   // 读管道的文件id
    int write_fd_ = -1;  // 写文件的文件id
};

}  // namespace sros

#endif  // MODULES_PIPE_PIPE_MODULE_H_
