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
add_executable(reference_parameter reference_parameter.cpp)
add_executable(array_form_parameter 6-function/array_form_parameter.cpp)

add_executable(initializer_list 6-function/initializer_list.cpp)

add_executable(array_point 6-function/array_point.cpp)

#add_executable(function_overload 6-function/function_overload.cpp)
#add_executable(special_language_feature 6-function/special_language_feature.cpp)

add_executable(constexpr_function 6-function/constexpr_function.cpp)

add_executable(debug_help 6-function/debug_help.cpp)

add_executable(function_match 6-function/function_match.cpp)

add_executable(function_pointer 6-function/function_pointer.cpp)

add_executable(for for.cpp)
add_executable(define_abstract_data_type 7-class/define_abstract_data_type.cpp)

add_executable(access_control_and_Encapsulation 7-class/access_control_and_Encapsulation.cpp)

add_executable(class_other_feature 7-class/class_other_feature.cpp)

add_executable(constructor_function_refind 7-class/constructor_function_refind.cpp)

add_executable(entrust_constructor_function 7-class/entrust_constructor_function.cpp)
add_executable(11111 11111.cpp)
add_executable(IO_class 8-IO_lib/IO_class.cpp)
add_executable(IO_file 8-IO_lib/IO_file.cpp)
add_executable(IO_string 8-IO_lib/IO_string.cpp)
add_executable(container_lib_overview 9-order_container/container_lib_overview.cpp)

add_executable(order_container_operator 9-order_container/order_container_operator.cpp)

add_executable(vector_obj_how_to_increase 9-order_container/vector_obj_how_to_increase.cpp)
add_executable(string_other_operator 9-order_container/string_other_operator.cpp)

add_executable(container_adapter 9-order_container/container_adapter.cpp)

add_executable(acquaintance_generic_algorithms 10-generic_algorithms/acquaintance_generic_algorithms.cpp)

add_executable(customize_operator 10-generic_algorithms/customize_operator.cpp)
add_executable(revisit_iterator 10-generic_algorithms/revisit_iterator.cpp)

add_executable(iostream_iterator 10-generic_algorithms/iostream_iterator.cpp)

add_executable(use_associate_containers 11-associative_containers/use_associate_containers.cpp)
add_executable(associate_containers_overview 11-associative_containers/associate_containers_overview.cpp)

add_executable(associate_containers_operator 11-associative_containers/associate_containers_operator.cpp)

add_executable(unorder_container 11-associative_containers/unorder_container.cpp)

add_executable(copy_assigment_destroyed 13-copy_control/copy_assigment_destroyed.cpp 13-copy_control/obj_move.cpp)

add_executable(copy_assigment_operator 13-copy_control/copy_assigment_operator.cpp)

add_executable(copy_control_sample 13-copy_control/copy_control_sample.cpp)

add_executable(dynamic_memory_management_class 13-copy_control/dynamic_memory_management_class.cpp)


#set(StrBlobPtr 12-dynamic_memory/head_file/StrBloPtr.hpp)
add_library(StrBlob  SHARED 12-dynamic_memory/StrBlob.cpp )
add_executable(dynamic_memory_and_technique_ptr 12-dynamic_memory/dynamic_memory_and_technique_ptr.cpp)
target_link_libraries(dynamic_memory_and_technique_ptr StrBlob )

add_executable(dynamic_arrays 12-dynamic_memory/dynamic_arrays.cpp)

add_library(TextQuery SHARED 12-dynamic_memory/TextQuery.cpp)
add_executable(main_node 12-dynamic_memory/main.cpp)
target_link_libraries(main_node TextQuery)




#add_executable(define_abstract_data_type 6-function/define_abstract_data_type.cpp)
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


