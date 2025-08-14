# NSL-Sample-Application

- NSL-3130AA / NSL-1110AA 모델의 WINDOWS / LINUX 용 SAMPLE CODE입니다.
- OPENCV와 연동하여 화면을 표시하므로 OPENCV의 압축을 해제하여 PATH를 설정해 주세요.
- Lidar 데이터를 통한 딥러닝을 테스트 하는 경우 main.cpp의 SUPPORT_DEEPLEARNING을 활성화 하십시오.
- 응용 프로그램의 옵션(argument)을 확인하고 싶은 경우 videoSource.cpp의 initAppCfg()를 참조 하십시오.
- 화면의 키보드 이벤트를 사용하여 모드 변경이 가능합니다. NSL3130AA.cpp 의 keyProc() 을 참조 하십시오.
- WINDOWS의 경우 POINT CLOUD를 위해 PCL-1.12.0-AllInOne-msvc2019-win64.exe 를 사용합니다.
- 제공되는 OpenCV4.5.4 버전의 설치 경로와 visual studio project 속성의 c++ 및 링커 탭의 패스를 맞추어 주세요.
- 필요에 따라 PCL DLL 경로를 환경변수에 등록 후 사용하세요.(EXE 파일로 실행하는 경우 DLL 경로 필요)
  - PATH=C:\Program Files\PCL 1.12.0\bin;C:\Program Files\OpenNI2\Redist;%OPENCV_DIR%bin;%PATH%;

## WINDOWS 컴파일 방법
- Visual studio 2019에서 테스트 되었습니다.
- PCL-1.12.0-AllInOne-msvc2019-win64.exe 및 opencv 를 먼저 설치 후 사용 하십시오.

## USB 인식용 rules 정의
```
$ sudo vi /etc/udev/rules.d/defined_lidar.rules
KERNEL=="ttyACM*", ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="0094", MODE:="0777",SYMLINK+="ttyLidar"

$ service udev reload
$ service udev restart
```

## LINUX 컴파일 방법
```
$ cd NSL-Sample-Application/NSL-Sample-Application/
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./nsl-dev or ./nsl-dev -captureType 1 ipaddr 192.168.0.220 or ./nsl-dev ipaddr /dev/ttyLidar or ./nsl-dev -help
```
