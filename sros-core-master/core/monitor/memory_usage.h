//
// Created by lhx on 17-12-13.
//

#ifndef SROS_MEMORY_USAGE_H
#define SROS_MEMORY_USAGE_H

#include <cstddef>
#include <string>

int total_memory_usage(size_t& total_ram_size, size_t & avail_ram_size);

size_t self_memory_usage();

size_t self_memory_usage_rss();

size_t self_memory_usage(const std::string &label);


#endif //SROS_MEMORY_USAGE_H
