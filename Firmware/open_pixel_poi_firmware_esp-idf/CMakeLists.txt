cmake_minimum_required(VERSION 3.16)
set(PARTITION_TABLE_CSV "${CMAKE_CURRENT_SOURCE_DIR}/partitions.csv")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(c3_mini_blink)
