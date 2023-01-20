cmake_minimum_required(VERSION 3.0.2)
project(exercise LANGUAGES CXX)

#创建目标---静态库
#add_library(message
#STATIC
#include/message.hpp
#message.cpp
#)

#创建可执行文件
#add_executable(hello_world hello_world.cpp)
add_executable(1 1.cpp)
add_executable(exercise2 exercise2.cpp)
add_executable(exercise3 exercise3.cpp)
add_executable(exercise4 exercise4.cpp)
add_executable(exercise5 exercise5.cpp)
add_executable(exercise_2.5.3 exercise_2.5.3.cpp)
add_executable(use_Sales_data use_Sales_data.cpp)
add_executable(string string.cpp)
add_executable(string1 string1.cpp)
add_executable(Vector Vector.cpp)
add_executable(iterator iterator.cpp)
add_executable(iterator1 iterator1.cpp)
add_executable(iterator2 iterator2.cpp)
add_executable(inbuilt_array inbuilt_array.cpp)
add_executable(inbuilt_array1 inbuilt_array1.cpp)
add_executable(inbuilt_array2 inbuilt_array2.cpp)
add_executable(expression expression.cpp)
add_executable(sizeof sizeof.cpp)
add_executable(intergral_promotion intergral_promotion.cpp)
add_executable(switch switch.cpp)
add_executable(exception exception.cpp)
#add_executable(pointer_form_parameter pointer_form_parameter.cpp)
add_executable(const_reference 6-function/const_reference.cpp)
add_executable(array_form_parameter 6-function/array_form_parameter.cpp)

add_executable(initializer_list 6-function/initializer_list.cpp)
add_executable(reference 6-function/reference.cpp)

add_executable(array_pointer 6-function/array_pointer.cpp)

add_executable(function_overload 6-function/function_overload.cpp)
add_executable(function_pointer 6-function/function_pointer.cpp)
add_executable(example 7-class/example.cpp)

add_executable(IO_class 8-IO_library/IO_class.cpp)
add_executable(IO_file 8-IO_library/IO_file.cpp)
add_executable(IO_string 8-IO_library/IO_string.cpp)

add_executable(order_container_overview 9-order_container/order_container_overview.cpp)
add_executable(order_container_operator 9-order_container/order_container_operator.cpp)
add_executable(overview 10-generic_algorithms/overview.cpp)

add_executable(accquaintance_generic_algrithms 10-generic_algorithms/accquaintance_generic_algrithms.cpp)



#将目标库链接到可执行目标
#target_link_libraries(hello_world message)



########################用条件语句控制编译###############################

#引入新变量USE_LIBBRAY ，值为OFF
#set(USE_LIBBRAY OFF)
#message(STATUS "Compile sources into a library? ${USE_LIBRARY}")  #打印引入新变量USE_LIBBRAY的值

#定义全局变量BUILD_SHARED_LIBS,并设置为OFF
#set(BUILD_SHARED_LIBS OFF)

#引入变量_source，包括 message.hpp message.cpp
#list(APPEND _sources message.cpp message.hpp)

#$if(USE_LIBRARY)
# add_library(message ${_sources})
 #add_executable(hello_world hello_world.cpp)
 #target_link_libraries(hello_world message)

#else()
 #add_executable(hello_world hello_world.cpp ${_sources})
 
#endif()









