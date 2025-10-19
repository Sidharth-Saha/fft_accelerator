cmake_minimum_required(VERSION 3.16)
project(fft_accelerator C)

# Use C11
set(CMAKE_C_STANDARD 11)
SET(CMAKE_C_STANDARD_REQUIRED ON)

add_library(fft_accelerator STATIC
	"src/fft_accelerator.c"
)

target_include_directories(fft_accelerator PUBLIC "include")

if(MSVC)
    target_compile_options(fft_accelerator PRIVATE /W4)
else()
    target_compile_options(fft_accelerator PRIVATE -Wall -Wextra -Wpedantic)
endif()