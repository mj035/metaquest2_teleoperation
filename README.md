# vr_teleoperation
host : ubuntu 22.04 ros2 humble
docker : ros1 noetic container
메타퀘스트 2의 quest2ros앱을 사용(ros1호환)-> ros1 도커 컨테이너에서 ros topic 받아옴 -> advanced bridge.py(docker) -> mujoco .py(host)
rostopic echo /q2r_left_hand_pose ---출력 값 보면 알겠지만 postion xyz + orientation xyzw 두개인데 postion+orientation 동시 고려해서 코드짜는게 넘 복잡하고
dual arm. xml ? 이거로 두팔로 띄워서 했었는데 이러면 넘 복잡해짐.그래서 그냥 single arm으로 함
구현 방법은 내가 원하는 위치로 metaquest2움직였을 때의 pose값 + joint 1234를 매칭한 데이터 확보(20~30개정도) 해서 이걸 코드에 박아두고
kd - tree 방식이 나도 전문적으로 알지는 못하는데, 그냥 내가 메타퀘스트 움직였을 때 출력되는 pose값 보고 그 pose값이랑 가장 가까운 우리가 박아둔 데이터에서 3~4개 데이터를 
빠르게 찾는 방식이라고 함 그래서 delay없이 실시간으로 텔레오퍼레이션이 가능한거고 쩃든 3~4개 데이터의 joint값들의 평균치를 계산해서 그 joint값으로 mujoco에서 로봇팔이 회전하도록 해서
기본적인 구현은 했음 


--

근데 코드 잘 보면 orientation값도 적용이 안된 거 같기도하고 joint4번 손목부근 관절 메타퀘스트 꺽으면 똑같이 꺽이는거 구현하려고 해봤는데 잘 안됌. 그래서 좀 더 수정하면 더 좋아질듯 
