# Минимальная версия CMake, необходимая для ESP-IDF
cmake_minimum_required(VERSION 3.16)

# Включаем стандартные макросы сборки ESP-IDF
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Название проекта (должно совпадать с именем папки)
project(esp-navigator)
