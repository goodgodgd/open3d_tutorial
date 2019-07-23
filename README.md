# open3d_tutorial



## Build Library


1. 저장소 받기 (**--recursive** 옵션에 주의)

   ```
   git clone --recursive https://github.com/intel-isl/Open3D
   cd Open3D
   ```

2. 의존성 설치

   ```
   sudo ./util/scripts/install-deps-ubuntu.sh
   sudo apt install cmake-qt-gui
   ```

3. cmake-gui 실행

   ```
   mkdir build
   cd build
   cmake-gui ..
   ```

4. "Configure" and "Finish" 클릭

5. "BUILD_xxx" 옵션 세팅

   1. Uncheck **PYBIND11** and **PYTHON_MODULE**
   2. Check **SHARED_LIBS**, **TINYFILEDIALOGS** and **EIGEN3**

6. "CMAKE_INSTALL_PREFIX" 에 라이브러리 설치 경로 지정

7. "Configure", "Generate" 누르고 cmake-gui 나가기

8. 빌드 및 설치

   ```
   make -j3
   make install
   cp lib/* /path/to/install/lib/
   ```



## Import Open3D to QtCreator Project

QtCreator에서 프로젝트를 만든 후 `.pro` 파일에 다음 스크립트 추가

```c
INCLUDEPATH += <install path>/open3d/include \
                <install path>/open3d/include/Open3D/3rdparty/fmt/include \
                <install path>/open3d/include/Open3D/3rdparty/Eigen

LIBS += -L<install path>/open3d/lib \
        -lOpen3D
```





