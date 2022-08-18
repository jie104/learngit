//
// Created by lfc on 19-1-28.
//

#include "ifm3d_driver.hpp"

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = 0;
  FLAGS_logbufsecs = 0;
  google::SetStderrLogging(google::INFO);
  google::InstallFailureSignalHandler();

  std::shared_ptr<ifm::Ifm3dDriver> driver;
  driver.reset(new ifm::Ifm3dDriver);
  driver->init();
  //    driver.init();
  driver->getFrameLoop();
  return 0;
}