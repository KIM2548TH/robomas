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

# Import functions from beer_save_to_csv
from beer_save_to_csv import (
    sub_info_position, 
    sub_info_attitude, 
    sub_info_imu, 
    sub_info_esc, 
    sub_info_status,
    save_data_to_csv
)

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    # เริ่มเก็บข้อมูลเซ็นเซอร์
    print("เริ่มเก็บข้อมูลเซ็นเซอร์...")
    ep_chassis.sub_position(freq=1, callback=sub_info_position)
    ep_chassis.sub_attitude(freq=5, callback=sub_info_attitude)
    ep_chassis.sub_imu(freq=10, callback=sub_info_imu)
    ep_chassis.sub_esc(freq=20, callback=sub_info_esc)
    ep_chassis.sub_status(freq=50, callback=sub_info_status)

    # ทำลูป 4 รอบ
    for i in range(4):
        print(f"เริ่มรอบที่ {i+1}")
        
        # 1. เดินไปข้างหน้า 0.6 เมตร
        print("เดินไปข้างหน้า 0.6 เมตร")
        ep_chassis.move(x=0.6, y=0, z=0, xy_speed=0.7).wait_for_completed()
        
        # 2. หยุดและล็อคมอเตอร์ (ตั้งความเร็วเป็น 0)
        print("หยุดและล็อคมอเตอร์")
        ep_chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(2)  # หยุดพัก 2 วินาที
        
        # 3. เลี้ยวขวา 90 องศา
        print("เลี้ยวขวา 90 องศา")
        try:
            ep_chassis.move(x=0, y=0, z=-90, z_speed=30).wait_for_completed()
            print("เลี้ยวเสร็จแล้ว")
        except Exception as e:
            print(f"เกิดข้อผิดพลาดในการเลี้ยว: {e}")
            
        # หยุดและล็อคมอเตอร์อีกครั้งหลังเลี้ยว
        print("หยุดและล็อคมอเตอร์หลังเลี้ยว")
        ep_chassis.drive_speed(x=0, y=0, z=0)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  # หยุดล้อทั้งหมดอย่างชัดเจน
        time.sleep(2)  # หยุดพัก 2 วินาที
        
        print(f"เสร็จสิ้นรอบที่ {i+1}\n")

    # หยุดเก็บข้อมูลเซ็นเซอร์
    print("หยุดเก็บข้อมูลเซ็นเซอร์...")
    ep_chassis.unsub_status()
    ep_chassis.unsub_esc()
    ep_chassis.unsub_imu()
    ep_chassis.unsub_attitude()
    ep_chassis.unsub_position()

    # ล็อคมอเตอร์ครั้งสุดท้าย
    print("ล็อคมอเตอร์ครั้งสุดท้ายและปิดการเชื่อมต่อ")
    ep_chassis.drive_speed(x=0, y=0, z=0)
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  # หยุดล้อทั้งหมดอย่างชัดเจน
    time.sleep(1)  # รอให้หยุดสนิท

    # บันทึกข้อมูลลง CSV
    print("บันทึกข้อมูลลง CSV...")
    save_data_to_csv()
    
    ep_robot.close()
    print("เสร็จสิ้นการทำงาน!")