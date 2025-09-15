# speed-stop-code
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy # ros 노드 생성, 토픽, 서비스 호출 가능케 함.
from moari_msgs.msg import CtrlCmd, CollisionData, EgoVehicleStatus, EventInfo 
# 차량 제어 명령, 충돌 정보, 차량 상태 정보, 이벤트 관련 데이터
from moari_msgs.srv import MoraiEventCmdSrv
# morai 시뮬레이터와 서비스 통신을 하기 위한 타입 불러오기
from enum import Enum

class Gear(Enum):
    P = 1
    R = 2
    N = 3
    D = 4
    
class s_drive():
    def __init__(self):
        rospy.init_node('collision_avoid', anonymous=True)
        # ros 노드 초기화

        # publisher
        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        # subscriber
        rospy.Subscriber('/CollisionData', CollisionData, self.collision_callback)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)

        # service
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_cmd_srv = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)

        
        self.rate = rospy.Rate(10)

        self.is_collision = False
        self.ego_status = EgoVehicleStatus()

        # 처음에 auto_mode , drive gear로 세팅
        self.send_gear_cmd(Gear.D.value)

        while not rospy.is_shutdown():
            if self.is_collision:
                # 충돌 발생 시 후진 기어
                self.send_gear_cmd(Gear.R.value)

                for _ in range(20):
                    self.send_ctrl_cmd(0.0, 10)   # (steering, velocity)
                    self.rate.sleep()

                self.send_gear_cmd(Gear.D.value)

            else:
                self.send_ctrl_cmd(0.0, 10)
                self.rate.sleep()

    # 충돌 메시지 콜백 함수
    def collision_callback(self, data):
        if len(data.collision_object) > 0:
            self.is_collision = True
            rospy.loginfo("Collision detected!")
        else:
            self.is_collision = False

    # Ego 차량 상태 정보 콜백 함수
    def ego_callback(self, data):
        self.ego_status = data
        rospy.loginfo(f"Velocity: {self.ego_status.velocity.x * 3.6:.2f} km/h")

    # 기어 변경 이벤트 메시지 세팅 함수
    def send_gear_cmd(self, gear_mode):
        # 기어 변경 전 차량 속도가 0에 가까워야 함
        while abs(self.ego_status.velocity.x) > 0.1:
            self.send_ctrl_cmd(0, 0)
            self.rate.sleep()

        gear_cmd = EventInfo()
        gear_cmd.option = 3
        gear_cmd.ctrl_mode = 3
        gear_cmd.gear = gear_mode
        gear_cmd_resp = self.event_cmd_srv(gear_cmd)
        rospy.loginfo(f"Gear change: {gear_cmd}")

    # ctrl_cmd 메시지 세팅 함수
    def send_ctrl_cmd(self, steering, velocity):
        cmd = CtrlCmd()
        if velocity > 0:
            cmd.longlCmdType = 2
            cmd.velocity = velocity
            cmd.steering = steering
        else:
            cmd.longlCmdType = 1
            cmd.brake = 1
            cmd.steering = 0
        self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    try:
        s_d = s_drive()
    except rospy.ROSInterruptException:
        pass
