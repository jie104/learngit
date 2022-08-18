#!/bin/bash
# file matrix_build.sh
# author perry
# date 22-06-30.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe 打包matrix
# $1 == "release" //发布版本

cd ../update/ && pwd
MATRIX_DIR='../../chip-web'

# 若是发布版本就打开下面开关，这样发布版本就会带有chip-web的功能，若不是发布版本就用以前老的chip-web版本。
release_version=false
if [[ $1 == 'release' ]]; then
  release_version=true
fi

function get_gitlab_matrix_artifacts() {
  # 先查下匹配的分支
  if [[ $CI_JOB_ID == "" ]]; then # 不是用gitlab打包
    SROS_CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
  else
    SROS_CURRENT_BRANCH=${CI_COMMIT_REF_NAME}
  fi
  MATRIX_PACKAGE_BRANCH=$(grep -m 1 "${SROS_CURRENT_BRANCH}" ../package.csv | awk -F ',' '{print $2}')
  if [[ $MATRIX_PACKAGE_BRANCH == "" ]]; then # 若没有匹配的分支就查匹配的版本号
    source ../version.sh
    MATRIX_PACKAGE_BRANCH=$(grep -m 1 "${sros_version_str}" ../package.csv | awk -F ',' '{print $2}')
    if [[ $MATRIX_PACKAGE_BRANCH == "" ]]; then
      echo -e "\033[31m 未找到需要打包的版本号！ \033[0m"
      exit 1
    fi
  fi
  if [[ -f artifacts-matrix-SAP.zip ]]; then
    rm artifacts-matrix-SAP.zip
  fi
  if [ -d dist ]; then
    rm dist -r
  fi
  # 下载Matrix打包后的数据
  echo "curl --output artifacts-matrix-SAP.zip 'http://git.standard-robots.com/api/v4/projects/225/jobs/artifacts/${MATRIX_PACKAGE_BRANCH}/download?private_token=NmQ2y_g6tu7WfezwsEVV&job=Single-Page-Application'"
  if ! curl --output artifacts-matrix-SAP.zip "http://git.standard-robots.com/api/v4/projects/225/jobs/artifacts/${MATRIX_PACKAGE_BRANCH}/download?private_token=NmQ2y_g6tu7WfezwsEVV&job=Single-Page-Application"; then
    echo -e "\033[31m 从gitlab下载Matrix打包文件失败！ \033[0m"
    exit 1
  fi
  # 解压Matrix升级包
  if ! unzip artifacts-matrix-SAP.zip; then
    echo -e "\033[31m 解压Matrix升级包失败，可能是不存在对应的Matrix升级包。 \033[0m"
    exit 1
  fi
}

if $release_version; then # 需要打包Matrix
  if [[ $CI_JOB_ID == "" ]]; then # 不是用gitlab打包
    # 将相对路径转换为绝对路径
    cd $MATRIX_DIR && {
      if ! "./build.sh"; then
        echo "\033[31m 打包Matrix失败！ \033[0m"
        exit 1
      fi
      MATRIX_DIST_DIR=$(pwd) # 获取绝对路径
      cd - || exit 1
    }
  else # 用ci打包
    get_gitlab_matrix_artifacts
    MATRIX_DIST_DIR="$(pwd)"
  fi

  echo "matrix_build : $(pwd)"
  echo "MATRIX_DIST_DIR: ${MATRIX_DIST_DIR}"
  if [ -d "${MATRIX_DIST_DIR}" ]; then
    # 拷贝chip-web到ui-server目录
    cp -rL ../web content/sros/update/
    cp -r ${MATRIX_DIST_DIR}/dist/ content/sros/update/web/ui-server/static/chip-web/
    cp content/sros/update/web/ui-server/static/chip-web/spa-mat/index.html content/sros/update/web/ui-server/templates/index.html
  fi
fi
