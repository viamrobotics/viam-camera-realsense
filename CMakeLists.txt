cmake_minimum_required(VERSION 3.25 FATAL_ERROR)
set(CMAKE_PROJECT_VERSION 0.0.1)


project(viam-camera-realsense
    DESCRIPTION "Viam Module for the Intel RealSense Camera"
    HOMEPAGE_URL https://github.com/viam-modules/viam-camera-realsense
    LANGUAGES CXX
)

include(cmake/simd_flags.cmake)

option(VIAM_REALSENSE_ENABLE_TESTS "Enable unit tests" OFF)
option(VIAM_REALSENSE_ENABLE_COVERAGE "Enable code coverage" OFF)

if(VIAM_REALSENSE_ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        add_compile_options(--coverage -O0 -g)
        add_link_options(--coverage)
    endif()

    # Runs lcov over the .gcda files of a prior ctest run. It does not run the
    # tests itself, so CI can run the suites as separate steps. Build the `all`
    # target and run ctest first.
    find_program(LCOV_EXECUTABLE lcov REQUIRED)
    add_custom_target(coverage
        COMMAND ${LCOV_EXECUTABLE} --capture --directory . --output-file coverage.info
        COMMAND ${LCOV_EXECUTABLE} --remove coverage.info /usr/* */test/* */_deps/* */.conan2/* --output-file coverage.info
        COMMAND ${LCOV_EXECUTABLE} --list coverage.info
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        VERBATIM
    )
endif()

if (APPLE)
    list(PREPEND CMAKE_INSTALL_RPATH "@loader_path/../lib")
else()
    list(PREPEND CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(realsense2 REQUIRED)
find_package(viam-cpp-sdk REQUIRED)
find_package(libjpeg-turbo CONFIG REQUIRED)
find_package(CURL REQUIRED)
find_package(libzip REQUIRED)

add_library(viamrealsense
    src/module/encoding.cpp
    src/module/zip_utils.cpp
    src/module/download_utils.cpp
    src/module/firmware_update.cpp
)
target_include_directories(viamrealsense
    PUBLIC src/module

)

target_compile_features(viamrealsense PUBLIC
    cxx_std_17
)
target_link_libraries(viamrealsense
    PUBLIC
        viam-cpp-sdk::viamsdk
        realsense2::realsense2
        libjpeg-turbo::turbojpeg-static
        CURL::libcurl
        libzip::zip
    PRIVATE
        ${CMAKE_DL_LIBS}
)
add_executable(viam-camera-realsense src/module/main.cpp)
target_link_libraries(viam-camera-realsense PRIVATE viamrealsense)

install(TARGETS viam-camera-realsense DESTINATION .)
install(FILES meta.json 99-realsense-libusb.rules DESTINATION .)
install(PROGRAMS first_run.sh install_udev_rules.sh DESTINATION .)

# `cmake --build <dir> --target package` assembles module.tar.gz from the
# install rules above.
set(CPACK_PACKAGE_NAME "viam-camera-realsense")
set(CPACK_PACKAGE_FILE_NAME "module")
set(CPACK_GENERATOR "TGZ")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
include(CPack)

if (VIAM_REALSENSE_ENABLE_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
