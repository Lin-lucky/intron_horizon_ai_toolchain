cmake_minimum_required(VERSION 2.8)
project(02_advanced_samples)

if (J5)
    add_subdirectory(multi_stages)
endif()

add_subdirectory(custom_identity)
add_subdirectory(multi_model_batch)
