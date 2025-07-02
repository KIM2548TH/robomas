# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import time
import robomaster
from robomaster import robot


def sub_info_handler(sub_info):
    print("sub info: {0}".format(sub_info))


def move_forward(chassis, distance, speed):
    """เดินไปข้างหน้าระยะทางที่กำหนด"""
    print(f"เดินไปข้างหน้า {distance} เมตร")
    chassis.move(x=distance, y=0, z=0, xy_speed=speed).wait_for_completed(5)
    chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  # หยุดล้อทั้งหมดอย่างชัดเจน


def stop_and_lock_motors(chassis, sleep_time):
    """หยุดและล็อคมอเตอร์"""
    print("หยุดและล็อคมอเตอร์")
    chassis.drive_speed(x=0, y=0, z=0)
    chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  # หยุดล้อทั้งหมดอย่างชัดเจน
    time.sleep(sleep_time)  # หยุดพัก


def turn_right(chassis, angle, speed):
    """เลี้ยวขวาตามมุมที่กำหนด"""
    print(f"เลี้ยวขวา {angle} องศา")
    chassis.move(x=0, y=0, z=-angle, z_speed=speed).wait_for_completed(5)
    print("เลี้ยวเสร็จแล้ว")
    
    print("หยุดและล็อคมอเตอร์หลังเลี้ยว")
    chassis.drive_speed(x=0, y=0, z=0)
    chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  # หยุดล้อทั้งหมดอย่างชัดเจน
    time.sleep(2)  # หยุดพัก


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    # ทำลูป 4 รอบ
    for i in range(4):
        print(f"เริ่มรอบที่ {i+1}")
        
        # 1. เดินไปข้างหน้า 0.6 เมตร
        move_forward(ep_chassis, distance=0.6, speed=0.7)
        
        # 2. หยุดและล็อคมอเตอร์
        stop_and_lock_motors(ep_chassis, sleep_time=2)
        
        # 3. เลี้ยวขวา 90 องศา
        turn_right(ep_chassis, angle=90, speed=30)
        
        print(f"เสร็จสิ้นรอบที่ {i+1}\n")

    # ล็อคมอเตอร์ครั้งสุดท้าย
    print("ล็อคมอเตอร์ครั้งสุดท้ายและปิดการเชื่อมต่อ")
    ep_chassis.drive_speed(x=0, y=0, z=0)
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  # หยุดล้อทั้งหมดอย่างชัดเจน
    time.sleep(1)  # รอให้หยุดสนิท
    
    ep_robot.close()
    print("เสร็จสิ้นการทำงาน!")
