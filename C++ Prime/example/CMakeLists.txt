cmake_minimum_required(VERSION 3.0.2)
project(example)




# add_executable(example1 example1.cpp)
# add_executable(example2 example2.cpp)
# add_executable(example3 example3.cpp)
# add_executable(example4 example4.cpp)
# add_executable(example5 example5.cpp)
# add_executable(example7 example7.cpp)
# add_executable(example8 example8.cpp)
# add_executable(example9 example9.cpp)
# add_executable(example11 example11.cpp)
add_executable(example_13 example_13.cpp)
add_executable(string string.cpp)
add_executable(Vector Vector.cpp)
add_executable(iterator iterator.cpp)
add_executable(iterator1 iterator1.cpp)
# add_executable(inbuilt_array inbuilt_array.cpp)
add_executable(inbuilt_array2 inbuilt_array2.cpp)
add_executable(eigen eigen.cpp)
add_executable(expression expression.cpp)

add_executable(comma_operator comma_operator.cpp)

add_executable(intergral_promotion intergral_promotion.cpp)
add_executable(implicit_convert implicit_convert.cpp)
add_executable(statement statement.cpp)
add_executable(switch switch.cpp)
add_executable(exception exception.cpp)
add_executable(function_base 6-function/function_base.cpp)

add_executable(local_object 6-function/local_object.cpp)
# add_executable(decltype decltype.cpp)

#add_dependencies(head_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(use_hello ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(head_src
 # ${catkin_LIBRARIES}
#)

#target_link_libraries( use_hello
 # head_src
  #${catkin_LIBRARIES}
#)


