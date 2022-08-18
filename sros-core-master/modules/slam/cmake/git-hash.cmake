 ###
 # @file message_bag.hpp
 # @author zmy (626670628@qq.com)
 # @brief 获取git当前提交hash
 # @version 0.1
 # @date 2021-05-18
 # 
 # 
 ###

find_package(Git QUIET)


function(get_hash hash_value work_dir)

set(GIT_HASH "unknown")

if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:%h
    OUTPUT_VARIABLE GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    WORKING_DIRECTORY
        ${work_dir}
    )  
endif()

if(EXISTS ${work_dir} AND IS_DIRECTORY ${work_dir})
    set(${hash_value} ${GIT_HASH} PARENT_SCOPE)
elseif()
     message(STATUS "${work_dir} is not exists or is not a directory")
endif()

    
endfunction(get_hash)

