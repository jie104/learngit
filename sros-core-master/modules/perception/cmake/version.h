//
// Created by zx on 2020/8/20.
//

#ifndef VERSION_H
#define VERSION_H

#define PERCEPTION_VER_MAJOR 1
#define PERCEPTION_VER_MINOR 0
#define PERCEPTION_VER_PATCH 0

#define GET_STRING(n) GET_STRING_DIRECT(n)
#define GET_STRING_DIRECT(n) #n

#define PERCEPTION_VERSION_STRING                                        \
  GET_STRING(PERCEPTION_VER_MAJOR)                                       \
  "." GET_STRING(PERCEPTION_VER_MINOR) "." GET_STRING(                   \
      PERCEPTION_VER_PATCH)

#endif //VERSION_H
