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
import math

latest_gimbal_angle = [0, 0, 0, 0]
lastest_distance = [0, 0, 0, 0]

def sub_data_angle(angle_info):
    global latest_gimbal_angle
    latest_gimbal_angle = angle_info

def sub_data_distance(sub_info):
    global lastest_distance
    lastest_distance = sub_info

def move_gimbal(ep_gimbal,ep_chassis):
    way = [0,0,0]

    # หันซ้าย
    ep_gimbal.moveto(pitch=-10, yaw=-90, pitch_speed=200, yaw_speed=200).wait_for_completed()
    print("หันซ้าย",lastest_distance[0])
    if lastest_distance[0] < 200 :
        way[0] = 0 #ทางตัน
        ep_chassis.move(x=0, y=(200-lastest_distance[0])/1000, z=0, xy_speed=1.5).wait_for_completed()
    elif lastest_distance[0] < 400:
        way[0] = 0 #ทางตัน
        ep_chassis.move(x=0, y=((lastest_distance[0])-200)/1000, z=0, xy_speed=1.5).wait_for_completed()
    else:
        way[0] = 1 #ทางไกล

    # หันหน้า
    ep_gimbal.moveto(pitch=-10, yaw=0, pitch_speed=100, yaw_speed=100).wait_for_completed()
    print("หันกลาง",lastest_distance[0])
    if lastest_distance[0] < 200 :
        way[1] = 0 #ทางตัน
        ep_chassis.move(x=-(abs(200-lastest_distance[0])/1000), y=0, z=0, xy_speed=1.5).wait_for_completed()
    elif lastest_distance[0] < 400:    
        way[1] = 0 #ทางตัน
        ep_chassis.move(x=(abs(200-lastest_distance[0])/1000), y=0, z=0, xy_speed=1.5).wait_for_completed()
    else:
        way[1] = 1 #ทางไกล

    # หันขวา
    ep_gimbal.moveto(pitch=-10, yaw=90, pitch_speed=200, yaw_speed=200).wait_for_completed()
    print("หันขวา",lastest_distance[0])
    if lastest_distance[0] < 200 :
        way[2] = 0 #ทางตัน
        ep_chassis.move(x=0, y=-(abs(200-lastest_distance[0])/1000), z=0, xy_speed=1.5).wait_for_completed()
    elif lastest_distance[0] < 400:    
        way[2] = 0 #ทางตัน
        ep_chassis.move(x=0, y=(abs(200-lastest_distance[0])/1000), z=0, xy_speed=1.5).wait_for_completed()   
    else:
        way[2] = 1 #ทางไกล

    # หันกลับด้านหน้า
    ep_gimbal.moveto(pitch=-10, yaw=0, pitch_speed=200, yaw_speed=200).wait_for_completed()
    return way


def convert_gimbal_result_to_blocked_directions(way_result, current_facing_direction='x+'):
    """
    แปลงผลลัพธ์จาก move_gimbal เป็น blocked directions ตามทิศทางที่หุ่นยนต์หันหน้าไป
    
    Args:
        way_result (list): [ซ้าย, หน้า, ขวา] (0=ทางตัน, 1=ทางว่าง) จาก move_gimbal  
        current_facing_direction (str): ทิศทางที่หุ่นยนต์หันหน้าไป
    
    Returns:
        list: รายการทิศทางที่เป็นทางตัน เช่น ['x-', 'y+']
    """
    blocked_directions = []
    
    # แมปทิศทางตามการหันหน้าของหุ่นยนต์
    direction_map = {
        'y+': {'left': 'x-', 'front': 'y+', 'right': 'x+'},  # หันหน้าไปเหนือ
        'x+': {'left': 'y+', 'front': 'x+', 'right': 'y-'},  # หันหน้าไปตะวันออก
        'y-': {'left': 'x+', 'front': 'y-', 'right': 'x-'},  # หันหน้าไปใต้
        'x-': {'left': 'y-', 'front': 'x-', 'right': 'y+'}   # หันหน้าไปตะวันตก
    }
    
    direction_names = {
        'x+': 'ทิศตะวันออก', 'x-': 'ทิศตะวันตก', 
        'y+': 'ทิศเหนือ', 'y-': 'ทิศใต้'
    }
    
    # ตรวจสอบแต่ละทิศทาง
    positions = ['left', 'front', 'right']
    position_names = ['ซ้าย', 'หน้า', 'ขวา']
    
    print(f"🔍 วิเคราะห์ผลการสแกน:")
    print(f"   - หันหน้าไปทิศ: {current_facing_direction}")
    print(f"   - ผลสแกน [ซ้าย, หน้า, ขวา]: {way_result}")
    print(f"   - (0=ทางตัน, 1=ทางว่าง)")
    
    for i, position in enumerate(positions):
        direction_key = direction_map[current_facing_direction][position]
        
        if way_result[i] == 0:  # ทางตัน
            blocked_directions.append(direction_key)
            print(f"🚫 {position_names[i]} ทางตัน -> {direction_key} ({direction_names[direction_key]})")
        else:  # ทางว่าง
            print(f"✅ {position_names[i]} ทางว่าง -> {direction_key} ({direction_names[direction_key]})")
    
    return blocked_directions

def detect_walls_with_gimbal(ep_robot, current_facing_direction='x+'):
    """
    ฟังก์ชันหลักสำหรับตรวจสอบกำแพงด้วย gimbal และแปลงเป็น blocked directions
    
    Args:
        ep_robot: robot object ที่ initialized แล้ว
        current_facing_direction (str): ทิศทางที่หุ่นยนต์หันหน้าไป
    
    Returns:
        list: รายการทิศทางที่เป็นทางตัน เช่น ['x-', 'y+']
    """
    print(f"🔍 ตรวจสอบกำแพงด้วย gimbal (หันหน้าไป: {current_facing_direction})")
    
    # Initialize เซ็นเซอร์
    ep_gimbal, ep_sensor, ep_chassis = initialize_sensors(ep_robot)
    
    try:
        # ใช้ฟังก์ชัน move_gimbal ที่มีอยู่แล้ว
        way_result = move_gimbal(ep_gimbal, ep_chassis)
        print(f"📊 ผลการตรวจสอบ [ขวา, หน้า, ซ้าย]: {way_result}")

        # แปลงเป็น blocked directions
        blocked_directions = convert_gimbal_result_to_blocked_directions(way_result, current_facing_direction)
        
        if blocked_directions:
            print(f"🚫 ทางตันที่พบ: {blocked_directions}")
        else:
            print("✅ ไม่พบทางตัน")
        
        return way_result,blocked_directions
        
    finally:
        # ปิดเซ็นเซอร์
        cleanup_sensors(ep_gimbal, ep_sensor, ep_chassis)

def initialize_sensors(ep_robot):
    """
    ฟังก์ชันสำหรับ initialize การอ่านเซ็นเซอร์ทั้งหมด
    
    Args:
        ep_robot: robot object ที่ได้จาก robot.Robot()
    
    Returns:
        tuple: (ep_gimbal, ep_sensor, ep_chassis)
    """
    # เชื่อมต่อกับส่วนประกอบต่างๆ
    ep_gimbal = ep_robot.gimbal
    ep_sensor = ep_robot.sensor
    ep_chassis = ep_robot.chassis
    
    # subscribe ข้อมูลจากเซ็นเซอร์
    ep_gimbal.sub_angle(freq=10, callback=sub_data_angle)
    ep_sensor.sub_distance(freq=10, callback=sub_data_distance)
    
    # รอให้เซ็นเซอร์พร้อม
    time.sleep(0.5)
    
    print("เซ็นเซอร์พร้อมใช้งาน")
    
    return ep_gimbal, ep_sensor, ep_chassis

def cleanup_sensors(ep_gimbal, ep_sensor, ep_chassis):
    """
    ฟังก์ชันสำหรับปิดการอ่านเซ็นเซอร์
    """
    try:
        ep_gimbal.unsub_angle()
        ep_sensor.unsub_distance()
        print("ปิดเซ็นเซอร์เรียบร้อย")
    except Exception as e:
        print(f"เกิดข้อผิดพลาดในการปิดเซ็นเซอร์: {e}")

def check_walls_around(ep_robot):
    """
    ฟังก์ชันเก่าสำหรับ backward compatibility
    
    Args:
        ep_robot: robot object ที่ initialized แล้ว
    
    Returns:
        list: ผลการตรวจสอบทั้ง 3 ทิศทาง [ซ้าย, หน้า, ขวา]
    """
    # Initialize เซ็นเซอร์
    ep_gimbal, ep_sensor, ep_chassis = initialize_sensors(ep_robot)
    
    try:
        # ตรวจสอบทางไป
        way_result = move_gimbal(ep_gimbal, ep_chassis)
        
        return way_result
        
    finally:
        # ปิดเซ็นเซอร์
        cleanup_sensors(ep_gimbal, ep_sensor, ep_chassis)

# ตัวอย่างการใช้งาน
if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    try:
        # เคลื่อนที่ไปข้างหน้า
        ep_chassis = ep_robot.chassis
        ep_chassis.move(x=0.6, y=0, z=0, xy_speed=0.5).wait_for_completed()
        
        # ตรวจสอบกำแพงและแปลงเป็น blocked directions
        blocked_dirs = detect_walls_with_gimbal(ep_robot, 'y+')
        print("ทิศทางที่เป็นทางตัน:", blocked_dirs)
        
    finally:
        ep_robot.close()