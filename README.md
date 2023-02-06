# NSL-Sample-Application

- NSL-3130AA / NSL-1110AA 모델의 WINDOWS / LINUX 용 SAMPLE CODE입니다.
- OPENCV와 연동하여 화면을 표시하므로 OPENCV를 먼저 설치하셔야 합니다. 
- LINUX 버전은 2D viewer로 구현되어 있으며 WINDOWS 버전은 3D viewer(Point cloud)를 지원합니다. 
- WINDOWS의 경우 POINT CLOUD를 위해 PCL-1.8.1-AllInOne-msvc2017-win64.exe 를 사용합니다.
- 제공되는 OpenCV4.5.4 버전의 설치 경로와 visual studio project 속성의 c++ 및 링커 탭의 패스를 맞추어 주세요.
- 화면의 키보드 이벤트를 사용하여 모드 변경이 가능합니다. NSL3130AA.cpp 의 keyProc() 을 참조 하십시오.
- 해당 제품을 네트워크에 연결 후 사용 가능 하며 아래는 LINUX 에서 COMPILE 하기 위한 방법입니다.

## WINDOWS 컴파일 방법
- Visual stude 2019에서 테스트 되었습니다.
- PCL-1.8.1-AllInOne-msvc2017-win64.exe 및 opencv 를 설치 하셔야 합니다.


## LINUX 컴파일 방법
```
$ cd NSL-Sample-Application/NSL-Sample-Application/
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./nsl-dev or ./nsl-dev -captureType 1 ipaddr 192.168.0.220
```
