# CMakeLists.txt
#
# AI 파이프라인 C++ 실행파일 빌드 (~/my/AI_c/)
#   - cam    : 카메라(GStreamer/libcamerasrc) -> OpenCV -> ai_frame_shm
#   - drowsy : ai_landmark_shm -> EAR 계산 + 다단계 판정 -> ai_status_shm
#
# (eye_seeker.py, ai_shm.py 는 파이썬이라 빌드 대상 아님)
#
# 빌드:
#   cd ~/my/AI_c
#   mkdir -p build && cd build
#   cmake ..
#   make -j4
#
# 결과물: build/cam , build/drowsy

cmake_minimum_required(VERSION 3.10)
project(ai_c LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

# ai_shm.hpp 를 찾기 위한 include 경로 (상위 디렉토리 = 소스 루트)
include_directories(${CMAKE_SOURCE_DIR})

# POSIX shm(shm_open 등) 링크: 구형 glibc는 librt 필요, 최신은 불필요.
# find_library로 있으면 링크, 없으면 무시.
find_library(RT_LIB rt)

# ---------------------------------------------------------------------
# drowsy : OpenCV 불필요 (좌표 -> EAR -> 판정)
# ---------------------------------------------------------------------
add_executable(drowsy drowsy.cpp)
if(RT_LIB)
    target_link_libraries(drowsy ${RT_LIB})
endif()

# ---------------------------------------------------------------------
# cam : OpenCV 필요 (GStreamer 지원 빌드된 OpenCV여야 libcamerasrc 동작)
# ---------------------------------------------------------------------
find_package(OpenCV REQUIRED)
add_executable(cam cam.cpp)
target_include_directories(cam PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cam ${OpenCV_LIBS})
if(RT_LIB)
    target_link_libraries(cam ${RT_LIB})
endif()

# 참고: OpenCV가 GStreamer 지원으로 빌드됐는지 확인하려면
#   python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -i gstreamer
# 또는 C++에서 cv::getBuildInformation() 출력. "GStreamer: YES" 여야 함.
