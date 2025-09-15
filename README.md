# Mandol_ws
## GPS team

유승민, 최대승, 이현수

## Hardware team

박준성, 류승훈

## Camera team

이승민



## Journey to start

#### gps open source setting

```bash
sudo apt update
sudo apt install ros-humble-rtcm-msgs ros-humble-nmea-msgs
sudo apt install ros-humble-tf-transformations

```

#### f9p, f9r

https://github.com/olvdhrm/RTK_GPS_NTRIP


#### ntrip
https://github.com/SGroe/ntrip_client_ros2


ros2 bag  play  100_bag/ -l

colcon build --packages-select gps_to_utm



---

* f9r이 만든 백파일에서 /f9r/fix 정보를 사용하여 utm 좌표값이 담긴 csv 파일 생성 모듈
Mandol_ws/src/gps_to_utm/src/f9r_to_csv.py

ros2 run gps_to_utm f9r_to_csv

* utm 좌표값이 담긴 csv 파일을 png 파일로 생성하는 모듈
Mandol_ws/data/processed/csv_to_png.py

python3 data/processed/csv_to_png.py

* f9r, f9p hz 확인
ros2 topic hz /f9r/fix
ros2 topic hz /f9p/fix

---


1. 백파일 폴더 경로에서 백파일 재생
ros2 bag  play  100_bag/ --topics /f9p/fix /f9r/fix -l
hannibal@hannibal:~/Mandol_ws/rosbag/gps_bag_9_10$ ros2 bag play 90_bag_15hz --topics /f9p/fix /f9r/fix -l


1. f9p(전륜축), f9r(후륜축) 센서로 /azimuth_angle 실시간 발행 노드
/Mandol_ws/src/calc_yaw/calc_yaw/gps_yaw_calculator.cpp

ros2 run cal_azimuth_angle azimuth_angle_calculator_node

2. f9r이 생성하는 /f9r/fix 정보를 섭하여 /f9r_utm 실시간 발행 노드
Mandol_ws/src/cal_azimuth_angle/cal_azimuth_angle/azimuth_angle_calculator.cpp

ros2 run gps_to_utm f9r_to_utm

1. f9p가 생성하는 /f9p/fix 정보를 섭하여 /f9p_utm 실시간 발행 노드
ros2 run gps_to_utm f9p_to_utm

ros2 run gps_to_utm f9r_to_utm

4. csv를 보간하여 /csv_path를 발행하고, /csv_path랑 gps를 tf하는 노드
Mandol_ws/src/gps_to_utm/src/tf_gps_csv.py

ros2 run gps_to_utm tf_gps_csv.py

5. f9r, f9p의 roi와 roi 끝점을 발행하는 노드
Mandol_ws/src/path_planning/src/f9r_roi_path.cpp
Mandol_ws/src/path_planning/src/f9p_roi_path.cpp

ros2 run path_planning f9r_roi_path
ros2 run path_planning f9p_roi_path