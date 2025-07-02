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
from robomaster import robot


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    # ทำลูป 3 รอบ
    for i in range(3):
        print(f"เริ่มรอบที่ {i+1}")
        
        # 1. เดินไปข้างหน้า 0.6 เมตร
        print("เดินไปข้างหน้า 0.6 เมตร")
        ep_chassis.move(x=0.6, y=0, z=0, xy_speed=0.5).wait_for_completed()
        
        # 2. หยุดและล็อคมอเตอร์ (ตั้งความเร็วเป็น 0)
        print("หยุดและล็อคมอเตอร์")
        ep_chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(1)  # หยุดพัก 1 วินาที
        
        # 3. เลี้ยวขวา 90 องศา
        print("เลี้ยวขวา 90 องศา")
        ep_chassis.move(x=0, y=0, z=-90, z_speed=45).wait_for_completed()
        
        # หยุดและล็อคมอเตอร์อีกครั้งหลังเลี้ยว
        print("หยุดและล็อคมอเตอร์หลังเลี้ยว")
        ep_chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(1)  # หยุดพัก 1 วินาที
        
        print(f"เสร็จสิ้นรอบที่ {i+1}\n")

    # ล็อคมอเตอร์ครั้งสุดท้าย
    print("ล็อคมอเตอร์ครั้งสุดท้ายและปิดการเชื่อมต่อ")
    ep_chassis.drive_speed(x=0, y=0, z=0)
    
    ep_robot.close()
    print("เสร็จสิ้นการทำงาน!")
