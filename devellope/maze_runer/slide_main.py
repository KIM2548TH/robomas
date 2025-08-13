# -*-coding:utf-8-*-

# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import robomaster
from robomaster import robot
from collections import deque
import statistics  # **เพิ่ม: สำหรับ median filter**
import math
import numpy as np
import cv2

from controler.map import Graph  # **เพิ่ม: นำเข้า Graph**
from controler.movement_slide import (
    move_direction_pid, 
    move_direction_pid_wall, 
    sub_chassis_position, 
    sub_chassis_attitude,  # **เพิ่ม**
    correct_robot_orientation,  # **เพิ่ม**
    move_to_tile_center_from_walls
)

# --- ค่าคงที่และตัวแปร Global ---
STEP_SIZE = 0.6  # ระยะทางในการเดินแต่ละครั้ง (เมตร)

# Global variables to store the latest sensor data from subscriptions
lastest_distance = [0] 

# **เพิ่ม: Median Filter สำหรับ ToF sensor**
tof_readings = deque(maxlen=5)  # เก็บข้อมูล 5 ค่าล่าสุด
filtered_distance = [0]  # ข้อมูลหลังกรอง

# Global variables for DFS state
visited_nodes = set()
scan_memory = {}  # !! เพิ่ม: หน่วยความจำสำหรับเก็บผลการสแกน

# **เพิ่ม: Map System**
maze_graph = Graph()  # กราฟแมพ
coord_to_node_id = {}  # แมพพิกัดกับ node_id


class MarkerInfo:

    def __init__(self, x, y, w, h, info):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._info = info

    @property
    def pt1(self):
        return int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)

    @property
    def pt2(self):
        return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)

    @property
    def center(self):
        return int(self._x * 1280), int(self._y * 720)

    @property
    def text(self):
        return self._info


markers = []
def on_detect_marker(marker_info):
    x, y, w, h, info = marker_info[0]
    markers.append(MarkerInfo(x, y, w, h, info))
    print("marker:{0} x:{1}, y:{2}, w:{3}, h:{4}".format(info, x, y, w, h))

# --- ฟังก์ชัน Helper ---
def format_coords(coords):
    """จัดรูปแบบพิกัดทศนิยมเพื่อการแสดงผล"""
    return f"({coords[0]:.2f}, {coords[1]:.2f})"

def apply_median_filter(new_value):
    """
    ใช้ Median Filter กับข้อมูล ToF
    
    Args:
        new_value (float): ค่าใหม่จาก ToF sensor
    
    Returns:
        float: ค่าหลังผ่าน median filter
    """
    global tof_readings, filtered_distance
    
    # เพิ่มค่าใหม่เข้า buffer
    tof_readings.append(new_value)
    
    # ถ้ามีข้อมูลมากกว่า 3 ค่า ให้ใช้ median
    if len(tof_readings) >= 3:
        filtered_value = statistics.median(tof_readings)
    else:
        # ถ้าข้อมูลน้อย ใช้ค่าเฉลี่ย
        filtered_value = sum(tof_readings) / len(tof_readings)
    
    filtered_distance[0] = filtered_value
    
    # แสดงข้อมูลการกรอง (เฉพาะเมื่อค่าเปลี่ยนแปลงมาก)
    # if abs(new_value - filtered_value) > 50:
    #     # print(f"🔧 ToF Filter: raw={new_value:.0f}mm -> filtered={filtered_value:.0f}mm (buffer: {list(tof_readings)})")
    
    return filtered_value

# --- ฟังก์ชัน Callback สำหรับ Subscription ---
def sub_data_distance(sub_info):
    """Callback function to update the latest distance from the sensor with median filter."""
    global lastest_distance
    
    raw_distance = sub_info[0]
    
    # **ใช้ median filter ก่อนเก็บข้อมูล**
    filtered_value = apply_median_filter(raw_distance)
    lastest_distance[0] = filtered_value

def get_stable_distance_reading():
    """
    อ่านค่าระยะทางแบบ stable โดยรอให้ค่าคงที่
    
    Returns:
        float: ค่าระยะทางที่ stable
    """
    stable_readings = []

    # อ่านค่า 11 ครั้ง
    for i in range(11):
        stable_readings.append(lastest_distance[0])
        time.sleep(0.02)  # รอ 20ms
    
    # ใช้ median ของ 5 ค่า
    stable_value = statistics.median(stable_readings)
    
    # ตรวจสอบความเสถียร
    variance = max(stable_readings) - min(stable_readings)
    if variance > 100:  # ถ้าค่าผันแปรมากกว่า 10cm
        print(f"⚠️  ToF ไม่เสถียร: variance={variance:.0f}mm, readings={stable_readings}")
    
    return stable_value

def checkmarkers(i,marker,ep_vision,x,qua, adjust_distance,ep_chassis):
    global markers
    markers.clear()
    ep_gimbal.moveto(pitch=-360, yaw=x, pitch_speed=350, yaw_speed=350).wait_for_completed()
    ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
    stable_distance = get_stable_distance_reading()
    time.sleep(0.25)  # *เพิ่มเวลาให้เซ็นเซอร์ stable*

    if markers:
        print(f"📍 พบ Marker ที่ทิศทางซ้าย: {markers[0].text}")
        marker[i] = markers[0].text  # *บันทึกข้อมูล Marker*
        
        # ep_vision.unsub_detect_info(name="marker")
    else:
        # markers.clear()
        move_direction_pid_wall(ep_chassis, qua, (stable_distance-230)/1000)
        ep_gimbal.moveto(pitch=-4, yaw=x, pitch_speed=350, yaw_speed=350).wait_for_completed()
        ep_gimbal.moveto(pitch=-360, yaw=x, pitch_speed=350, yaw_speed=350).wait_for_completed()
        # ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)

        time.sleep(0.05)
        if markers:
            print(f"📍 พบ Marker ที่ทิศทางซ้าย: {markers[0].text}")
            marker[i] = markers[0].text  # *บันทึกข้อมูล Marker*
        else:
            print(f"📍 ไม่พบ Marker !!!!!!!!!!!!!!")
        
        # ep_chassis.move(x=-0.05, y=0, z=0, xy_speed=0.7).wait_for_completed()
        stable_distance = get_stable_distance_reading()
        move_direction_pid_wall(ep_chassis, qua, (stable_distance-200)/1000)
    
    ep_vision.unsub_detect_info(name="marker")

def detect_red_color(ep_camera):
    time.sleep(0.5)
    """
    ตรวจจับสีแดงจากภาพที่ได้จากกล้อง Robomaster
    Args:
        ep_camera (robomaster.camera): camera module ของ Robomaster
    Returns:
        bool: True หากพบสีแดง, False หากไม่พบ
    """
    img = ep_camera.read_cv2_image()
    if img is None:
        print("❌ ไม่สามารถอ่านภาพจากกล้องได้")
        return False

    # ลด noise
    img_blur = cv2.GaussianBlur(img, (5, 5), 0)

    # แปลงเป็น HSV
    hsv_img = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)

    # ช่วงสีแดงแบบกว้างขึ้น
    lower_red1 = np.array([0, 80, 80])    # Hue ต่ำ, เพิ่ม S/V ให้สูงขึ้น
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 80, 80])  # Hue สูง
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)

    final_mask = mask1 + mask2

    # เปิด/ปิดช่องว่างเล็กๆ ใน mask
    kernel = np.ones((5, 5), np.uint8)
    final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, kernel)
    final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel)

    # ตรวจจับ pixel สีแดง
    red_pixels = np.count_nonzero(final_mask)
    if red_pixels > 5000:  # ปรับ threshold ตามความเหมาะสม
        print(f"✅ พบสีแดง ({red_pixels} pixels)")
        return True
    else:
        print(f"❌ ไม่พบสีแดง ({red_pixels} pixels)")
        return False



# --- ฟังก์ชันสแกนที่ปรับปรุงด้วย Median Filter ---
def move_gimbal(ep_gimbal, ep_chassis, ep_vision, ep_camera):
    """
    ฟังก์ชันสแกน 4 ทิศทาง + Median Filter + ปรับทิศทางให้ตรง
    คืนค่า way, marker, tof_wall
    """
    way = [0, 0, 0, 0]         # [ซ้าย, หน้า, ขวา, หลัง]
    marker = ["No"] * 4        # [ซ้าย, หน้า, ขวา, หลัง]
    tof_wall = [None] * 4      # [ซ้าย, หน้า, ขวา, หลัง]

    print("🧭 ตรวจสอบและปรับทิศทางก่อนสแกน...")
    correct_robot_orientation(ep_chassis, target_yaw=0)
    time.sleep(0.05)

    # ทิศทางและข้อมูลสำหรับวนลูป
    scan_info = [
        {'yaw': -90, 'dir': 'y+', 'idx': 0, 'marker_yaw': -90},
        {'yaw': 0,   'dir': 'x+', 'idx': 1, 'marker_yaw': 0},
        {'yaw': 90,  'dir': 'y-', 'idx': 2, 'marker_yaw': 90},
        {'yaw': 180, 'dir': 'x-', 'idx': 3, 'marker_yaw': 180},
    ]
    for info in scan_info:
        ep_gimbal.moveto(pitch=-6, yaw=info['yaw'], pitch_speed=350, yaw_speed=350).wait_for_completed()
        time.sleep(0.05)
        stable_distance = get_stable_distance_reading()
        print(f"หัน {info['yaw']}°: stable={stable_distance:.0f}mm")
        idx = info['idx']
        # กำหนดเงื่อนไขกำแพง
        if stable_distance < 600:
            way[idx] = 0
            tof_wall[idx] = stable_distance / 1000  # mm -> m
            # checkmarkers(0,marker,ep_vision,-90, 'y+', adjust_distance,ep_chassis)
            # ... (ใส่โค้ด marker detection ตามที่คุณมี) ...
        else:
            way[idx] = 1
            tof_wall[idx] = None

    # หันกลับด้านหน้า
    ep_gimbal.moveto(pitch=-6, yaw=0, pitch_speed=500, yaw_speed=500).wait_for_completed()
    time.sleep(0.05)
    correct_robot_orientation(ep_chassis, target_yaw=0)

    print(f"📊 way: {way}")
    print(f"📊 tof_wall: {tof_wall}")
    print(f"📍 marker: {marker}")
    return way, marker, tof_wall


# --- ฟังก์ชันหลักในการสำรวจด้วย DFS ---
def explore_from(current_coords, ep_chassis, ep_gimbal, ep_vision,ep_camera):
    """
    ฟังก์ชัน DFS หลัก ทำงานแบบ Recursive เพื่อสำรวจจากพิกัดปัจจุบัน
    """
    global visited_nodes, scan_memory, maze_graph, coord_to_node_id

    # 1. ทำเครื่องหมายว่าเคยมาเยือนแล้ว
    visited_nodes.add(current_coords)

    # **สร้างหรือหาโหนดปัจจุบัน**
    key = (round(current_coords[0], 2), round(current_coords[1], 2))
    if key not in coord_to_node_id:
        node_id = maze_graph.add_node(key[0], key[1])
        coord_to_node_id[key] = node_id
    else:
        node_id = coord_to_node_id[key]
        print(f"🔍 พบโหนดเดิม: ID={node_id} ที่ {format_coords(key)}")

    # 2. !! ตรวจสอบหน่วยความจำก่อนทำการสแกน พร้อมสร้างแมพ
    if current_coords in scan_memory:
        # ถ้าเคยสแกนแล้ว ให้ใช้ข้อมูลเก่า
        available_ways = scan_memory[current_coords]
        print(f"\n🧠 กลับมาที่ {format_coords(current_coords)}, ใช้ข้อมูลสแกนเก่า: {available_ways}")
        
        # **บันทึกข้อมูลสแกนถ้ายังไม่มี**
        if not maze_graph.nodes[node_id].blocked_directions:
            maze_graph.add_blocked_direction_to_node(node_id, available_ways)
        
    else:
        # ถ้าเป็นที่ใหม่ ให้ทำการสแกนและบันทึกผล
        print(f"\n📍 มาถึงพิกัดใหม่: {format_coords(current_coords)}. เริ่มสแกน...")
        
        # **รีเซ็ต median filter buffer เมื่อเริ่มสแกนใหม่**
        tof_readings.clear()
        time.sleep(0.1)  # รอให้ buffer เติมข้อมูลใหม่

        available_ways,marker, tof_wall = move_gimbal(ep_gimbal, ep_chassis, ep_vision,ep_camera)
        move_to_tile_center_from_walls(ep_chassis, available_ways, marker, tof_wall, tile_size=0.4)
        correct_robot_orientation(ep_chassis, target_yaw=0)

        scan_memory[current_coords] = available_ways
        print(f"🔬 ผลการสแกนถูกบันทึก: {available_ways}")
        
        # **บันทึกข้อมูลสแกน**
        maze_graph.add_blocked_direction_to_node(node_id, available_ways)
        maze_graph.add_marker_direction_to_node(node_id, marker)


    # 3. นิยามทิศทางและการเปลี่ยนแปลงของพิกัด พร้อมมุมกิมบอล
    direction_map = [
        {'name': 'left',     'direction': 'y-', 'coords': (0, -STEP_SIZE), 'gimbal_yaw': -90},   # ซ้าย = y-, กิมบอล -90°
        {'name': 'forward',  'direction': 'x+', 'coords': (STEP_SIZE, 0), 'gimbal_yaw': 0},     # หน้า = x+, กิมบอล 0°
        {'name': 'right',    'direction': 'y+', 'coords': (0, STEP_SIZE), 'gimbal_yaw': 90},    # ขวา = y+, กิมบอล 90°
        {'name': 'backward', 'direction': 'x-', 'coords': (-STEP_SIZE, 0), 'gimbal_yaw': 180},  # หลัง = x-, กิมบอล 180°
    ]
    
    # 4. วนลูปสำรวจแต่ละทิศทางตามผลลัพธ์จาก available_ways
    for i, direction_info in enumerate(direction_map):
        if available_ways[i] == 1:
            d_coords = direction_info['coords']
            next_coords = (round(current_coords[0] + d_coords[0], 2), round(current_coords[1] + d_coords[1], 2))

            if next_coords not in visited_nodes:
                print(f"  -> 👣 สไลด์ {direction_info['name']} ไปยัง {format_coords(next_coords)}...")
                
                # **สร้างหรือหาโหนดปลายทาง**
                next_key = (round(next_coords[0], 2), round(next_coords[1], 2))
                if next_key not in coord_to_node_id:
                    next_node_id = maze_graph.add_node(next_key[0], next_key[1])
                    coord_to_node_id[next_key] = next_node_id
                else:
                    next_node_id = coord_to_node_id[next_key]
                
                # **เชื่อมเส้นทาง (ตรวจสอบว่าเชื่อมแล้วหรือยัง)**
                current_node_id = coord_to_node_id[key]
                if next_node_id not in maze_graph.nodes[current_node_id].connections:
                    distance = math.sqrt((next_coords[0] - current_coords[0])**2 + (next_coords[1] - current_coords[1])**2)
                    maze_graph.add_edge(current_node_id, next_node_id, distance)
                
                # **หันกิมบอลไปทางที่จะเดิน**
                gimbal_yaw = direction_info['gimbal_yaw']
                print(f"     🎯 หันกิมบอลไป {gimbal_yaw}° ({direction_info['name']})")
                ep_gimbal.moveto(pitch=-6, yaw=gimbal_yaw, pitch_speed=350, yaw_speed=350).wait_for_completed()
                time.sleep(0.1)
                
                # ใช้ PID สำหรับการเคลื่อนที่สำรวจ
                move_direction_pid(ep_chassis, direction_info['direction'], STEP_SIZE)
                # available_ways2, marker, tof_wall = move_gimbal(ep_gimbal, ep_chassis, ep_vision, ep_camera)
                # move_to_tile_center_from_walls(ep_chassis, available_ways, marker, tof_wall, tile_size=0.4)

                explore_from(next_coords, ep_chassis, ep_gimbal,ep_vision,ep_camera)

                print(f"  <- ⏪ สไลด์กลับจาก {format_coords(next_coords)} มายัง {format_coords(current_coords)}...")
                
                # **หันกิมบอลไปทางที่จะย้อนกลับ**
                reverse_direction_map = {'x+': 'x-', 'x-': 'x+', 'y+': 'y-', 'y-': 'y+'}
                reverse_gimbal_map = {'x+': 180, 'x-': 0, 'y+': -90, 'y-': 90}  # ทิศทางตรงข้าม
                
                reverse_direction = reverse_direction_map[direction_info['direction']]
                reverse_gimbal_yaw = reverse_gimbal_map[direction_info['direction']]
                
                print(f"     🔄 หันกิมบอลไป {reverse_gimbal_yaw}° (ย้อนกลับ)")
                ep_gimbal.moveto(pitch=-6, yaw=reverse_gimbal_yaw, pitch_speed=350, yaw_speed=350).wait_for_completed()
                time.sleep(0.1)
                
                # ใช้ PID สำหรับการเคลื่อนที่ย้อนกลับ
                move_direction_pid(ep_chassis, reverse_direction, STEP_SIZE)
                available_ways2, marker, tof_wall = move_gimbal(ep_gimbal, ep_chassis, ep_vision, ep_camera)
                move_to_tile_center_from_walls(ep_chassis, available_ways, marker, tof_wall, tile_size=0.4)
                correct_robot_orientation(ep_chassis, target_yaw=0)
                
                # **หันกิมบอลกลับด้านหน้าหลังเดินกลับเสร็จ**
                print(f"     🏠 หันกิมบอลกลับด้านหน้า (0°)")
                ep_gimbal.moveto(pitch=-6, yaw=0, pitch_speed=350, yaw_speed=350).wait_for_completed()
                time.sleep(0.1)
                
            else:
                print(f"  -- เส้นทาง {direction_info['name']} ไปยัง {format_coords(next_coords)} เคยไปแล้ว ข้ามไป...")
                
                # **เชื่อมเส้นทางถ้ายังไม่เชื่อม**
                next_key = (round(next_coords[0], 2), round(next_coords[1], 2))
                if next_key in coord_to_node_id:
                    current_node_id = coord_to_node_id[key]
                    next_node_id = coord_to_node_id[next_key]
                    
                    # ตรวจสอบว่าเชื่อมแล้วหรือยัง
                    if next_node_id not in maze_graph.nodes[current_node_id].connections:
                        distance = math.sqrt((next_coords[0] - current_coords[0])**2 + (next_coords[1] - current_coords[1])**2)
                        maze_graph.add_edge(current_node_id, next_node_id, distance)
    
    print(f"✅ สำรวจจาก {format_coords(current_coords)} ครบทุกแขนงแล้ว")
from robomaster import camera

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gimbal = ep_robot.gimbal
    ep_sensor = ep_robot.sensor
    ep_chassis = ep_robot.chassis
    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera

    print("===== 🤖 เริ่มการสำรวจแผนที่ด้วย DFS + PID Movement + ToF Median Filter + Map System =====")
    
    # เพิ่ม subscription สำหรับ PID และ attitude
    ep_sensor.sub_distance(freq=50, callback=sub_data_distance)  # **ToF กับ median filter**
    ep_chassis.sub_position(freq=50, callback=sub_chassis_position)
    ep_chassis.sub_attitude(freq=20, callback=sub_chassis_attitude)
    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_360P)  # **เริ่มสตรีมวิดีโอจากกล้อง**
    
    time.sleep(0.5)  # **เพิ่มเวลารอให้ median filter buffer เติม**

    print(f"🔧 Median Filter เริ่มต้นแล้ว (buffer size: {tof_readings.maxlen})")
    print(f"🗺️  Map System เริ่มต้นแล้ว")

    try:
        start_node = (0.0, 0.0)
        explore_from(start_node, ep_chassis, ep_gimbal, ep_vision, ep_camera)

        
        # **แสดงสรุปแมพหลังสำรวจเสร็จ**
        print("\n" + "="*60)
        print("🗺️  สรุปแมพที่สร้างขึ้น")
        print("="*60)
        print(f"📊 จำนวนโหนดทั้งหมด: {len(maze_graph.nodes)}")
        print(f"📊 จำนวนเส้นทางทั้งหมด: {sum(len(node.connections) for node in maze_graph.nodes.values()) // 2}")
        
        print("\n📍 รายละเอียดโหนด:")
        for node_id, node in maze_graph.nodes.items():
            connections = list(node.connections.keys())
            blocked = node.blocked_directions
            marker = node.marker_directions
            print(f"   โหนด {node_id}: ({node.x:.2f}, {node.y:.2f})")
            print(f"     - เชื่อมต่อกับ: {connections}")
            print(f"     - ข้อมูลสแกน: {blocked}")  # [y-, x+, y+, x-]
            print(f"     - ข้อมูลสแกน marker: {marker}")
        print("="*60)
        
    except Exception as e:
        print(f"เกิดข้อผิดพลาด: {e}")
    finally:
        print("\n===== ⏹️ การสำรวจเสร็จสิ้น ทำการปิดระบบ =====")
        ep_sensor.unsub_distance()
        ep_chassis.unsub_position()
        ep_chassis.unsub_attitude()
        
        ep_camera.stop_video_stream()
        time.sleep(1)
        ep_robot.close()