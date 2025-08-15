# -*-coding:utf-8-*-
import csv
import os
import time
import statistics
import math
from collections import deque

from pid import PID, TurnPID
from map import Graph
import math
import numpy as np
import cv2
import csv

from robomaster import robot

# --- Global Variables ---
latest_chassis_position = [0, 0, 0]
latest_chassis_attitude = [0, 0, 0]
lastest_distance = [0]
tof_readings = deque(maxlen=5)   # <-- แก้จาก list เป็น deque
filtered_distance = [0]
markers = []

LOG_CSV_PATH = "robot_log.csv"
log_csv_header_written = False


# ตัวแปร global สำหรับ PID และ tracking
move_pid_x = None
move_pid_y = None
turn_pid = None  # **เพิ่ม: PID สำหรับการหมุน**


# --- ค่าคงที่และตัวแปร Global ---
STEP_SIZE = 0.6  # ระยะทางในการเดินแต่ละครั้ง (เมตร)

# Global variables for DFS state
visited_nodes = set()
scan_memory = {}  # !! เพิ่ม: หน่วยความจำสำหรับเก็บผลการสแกน

# **เพิ่ม: Map System**
maze_graph = Graph()  # กราฟแมพ
coord_to_node_id = {}  # แมพพิกัดกับ node_id



def log_robot_data():
    global log_csv_header_written
    timestamp = time.time()
    position = list(latest_chassis_position)
    tof1 = filtered_distance[0]  # ใช้ค่าที่ผ่านฟิลเตอร์แล้ว
    # ... (ส่วนอื่นเหมือนเดิม)
    # รองรับ marker เป็น object หรือ string
    if markers:
        marker_last = markers[-1]
        marker_name = marker_last.text if hasattr(marker_last, "text") else str(marker_last)
    else:
        marker_name = ""
    yaw_gimbal = latest_chassis_attitude[0] if len(latest_chassis_attitude) > 0 else None

    write_header = not os.path.exists(LOG_CSV_PATH) or not log_csv_header_written
    with open(LOG_CSV_PATH, "a", newline="") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(["timestamp", "position", "tof1", "marker", "yaw_gimbal"])
            log_csv_header_written = True
        writer.writerow([
            timestamp,
            f"{position[0]:.3f},{position[1]:.3f}",
            tof1,
            marker_name,
            yaw_gimbal
        ])

def get_stable_distance_reading():
    """
    คืนค่าระยะทางที่ผ่าน median filter แล้ว (mm)
    """
    global filtered_distance
    return filtered_distance[0]

def apply_median_filter(new_value, buffer_size=7):
    global tof_readings, filtered_distance
    tof_readings.append(new_value)
    if len(tof_readings) > buffer_size:
        tof_readings.pop(0)
    if len(tof_readings) >= 3:
        filtered_value = statistics.median(tof_readings)
    else:
        filtered_value = sum(tof_readings) / len(tof_readings)
    filtered_distance[0] = filtered_value
    return filtered_value

def sub_data_distance(sub_info):
    global lastest_distance
    raw_distance = sub_info[0]
    filtered_value = apply_median_filter(raw_distance)
    lastest_distance[0] = filtered_value
    log_robot_data()

def sub_chassis_position(position_info):
    global latest_chassis_position
    latest_chassis_position = list(position_info)
    log_robot_data()

def sub_chassis_attitude(attitude_info):
    global latest_chassis_attitude
    latest_chassis_attitude = list(attitude_info)
    log_robot_data()

def sub_data_angle(angle_info):
    global latest_chassis_attitude
    # สมมติ angle_info = [pitch, yaw, ...]
    # ปรับตามข้อมูลจริงของ angle_info
    if len(angle_info) >= 2:
        latest_chassis_attitude[0] = angle_info[1]  # หรือปรับ index ตามจริง
    log_robot_data()

class MarkerInfo:
    def __init__(self, x, y, w, h, info):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._info = info
    @property
    def text(self):
        return str(self._info)

def on_detect_marker(marker_info):
    global markers
    x, y, w, h, info = marker_info[0]
    markers.clear()  # <<== เพิ่มบรรทัดนี้
    markers.append(MarkerInfo(x, y, w, h, info))
    log_robot_data()



def correct_robot_orientation( target_yaw=0):
    """
    ปรับให้หุ่นยนต์หันตรงตามเป้าหมาย
    
    Args:
        ep_chassis: chassis controller
        target_yaw (float): มุม yaw เป้าหมาย (องศา)
    """
    global turn_pid, latest_chassis_attitude
    
    # เริ่มต้น PID สำหรับการหมุน
    if turn_pid is None:
        turn_pid = TurnPID(kp=1.0*0.7, ki=0.1*0.7, kd=0)
    
    turn_pid.reset()
    
    current_yaw = latest_chassis_attitude[0]
    yaw_error = target_yaw - current_yaw
    
    # ปรับให้อยู่ในช่วง -180 ถึง 180
    while yaw_error > 180:
        yaw_error -= 360
    while yaw_error < -180:
        yaw_error += 360
    
    print(f"🧭 ปรับทิศทาง: ปัจจุบัน {current_yaw:.1f}° -> เป้าหมาย {target_yaw:.1f}° (ต้องหมุน {yaw_error:.1f}°)")
    
    # ถ้าผิดพลาดน้อยกว่า 2 องศา ไม่ต้องปรับ
    if abs(yaw_error) < 0.05:
        print("✅ ทิศทางถูกต้องแล้ว")
        return
    
    tolerance = 0.05  # ความคลาดเคลื่อนที่ยอมรับได้
    stable_count = 0
    max_iterations = 120
    iteration = 0
    
    while iteration < max_iterations:
        iteration += 1
        
        current_yaw = latest_chassis_attitude[0]
        
        # คำนวณ PID output
        turn_output, angle_error, p, i, d = turn_pid.compute(target_yaw, current_yaw)
        
        # จำกัดความเร็วการหมุน
        max_turn_speed = 100  # องศา/วินาที
        turn_speed = max(-max_turn_speed, min(max_turn_speed, turn_output))
        
        # แสดงความคืบหน้า
        if iteration % 10 == 0:
            print(f"   🔄 ปัจจุบัน: {current_yaw:.1f}° ผิดพลาด: {angle_error:.1f}° ความเร็ว: {turn_speed:.1f}°/s")
        
        # ตรวจสอบว่าถึงเป้าหมายแล้ว
        if abs(angle_error) < tolerance:
            stable_count += 1
            if stable_count >= 30:
                print("✅ ปรับทิศทางเสร็จ!")
                break
        else:
            stable_count = 0
        
        # สั่งหมุน chassis
        ep_chassis.drive_speed(x=0, y=0, z=turn_speed)
        time.sleep(0.01)
    
    # หยุดการหมุน
    ep_chassis.drive_speed(x=0, y=0, z=0)
    time.sleep(0.1)

def move_direction_pid(ep_chassis, direction, distance):
    """
    เคลื่อนที่ไปยังทิศทางที่กำหนดด้วย PID control (ไม่หมุนตัว - สไลด์)
    
    Args:
        ep_chassis: chassis controller
        direction (str): ทิศทางที่ต้องการเคลื่อนที่ ('x+', 'x-', 'y+', 'y-')
        distance (float): ระยะทางที่ต้องการเคลื่อนที่ (เมตร)
    
    Returns:
        float: ระยะทางที่เคลื่อนที่ได้จริง
    """
    global move_pid_x, move_pid_y, latest_chassis_position
    
    # แมปทิศทางกับการเคลื่อนที่
    direction_map = {
        'x+': {'x': 1, 'y': 0, 'name': 'ข้างหน้า (x+)'},     # ไปข้างหน้า
        'x-': {'x': -1, 'y': 0, 'name': 'ข้างหลัง (x-)'},    # ไปข้างหลัง
        'y+': {'x': 0, 'y': 1, 'name': 'ขวา (y+)'},          # ไปขวา
        'y-': {'x': 0, 'y': -1, 'name': 'ซ้าย (y-)'}         # ไปซ้าย
    }
    
    if direction not in direction_map:
        print(f"❌ ทิศทางไม่ถูกต้อง: {direction}")
        return 0
    
    dir_info = direction_map[direction]
    print(f"🚶 สไลด์{dir_info['name']} {distance:.3f}m")
    
    # เริ่มต้น PID controllers
    if move_pid_x is None:
        move_pid_x = PID(kp=1.0, ki=0.1, kd=0)
    if move_pid_y is None:
        move_pid_y = PID(kp=1.0, ki=0.1, kd=0)
    
    # รีเซ็ต PID
    move_pid_x.reset()
    move_pid_y.reset()
    
    # **แก้ไข: ใช้ list() แทน .copy()**
    start_pos = list(latest_chassis_position)
    
    # คำนวณตำแหน่งเป้าหมาย
    target_x = start_pos[0] + (distance * dir_info['x'])
    target_y = start_pos[1] + (distance * dir_info['y'])
    
    tolerance = 0.05  # 1cm
    stable_iterations = 0
    max_iterations = 150
    iteration = 0
    
    print(f"📍 เริ่มต้นที่: ({start_pos[0]:.3f}, {start_pos[1]:.3f})")
    print(f"🎯 เป้าหมาย: ({target_x:.3f}, {target_y:.3f})")
    
    while iteration < max_iterations:
        iteration += 1

        
        # คำนวณระยะทางปัจจุบัน
        dx = latest_chassis_position[0] - start_pos[0]
        dy = latest_chassis_position[1] - start_pos[1]
        current_distance = math.sqrt(dx*dx + dy*dy)
        
        # คำนวณ error ในแต่ละแกน
        error_x = target_x - latest_chassis_position[0]
        error_y = target_y - latest_chassis_position[1]
        total_error = math.sqrt(error_x*error_x + error_y*error_y)
        
        # แสดงความคืบหน้าทุกๆ 0.5 วินาที
        if iteration % 25 == 0:
            progress = min(100, (current_distance / distance) * 100) if distance > 0 else 100
            # print(f"   📊 {current_distance:.3f}m/{distance:.3f}m ({progress:.1f}%) error:{total_error:.3f}m")
        
        # ตรวจสอบว่าถึงเป้าหมายแล้วหรือไม่
        if total_error < tolerance:
            stable_iterations += 1
            if stable_iterations >= 30:
                print("✅ ถึงเป้าหมายแล้ว!")
                break
        else:
            stable_iterations = 0
        
        # คำนวณ PID output สำหรับแต่ละแกน
        pid_output_x, _, _, _, _, _ = move_pid_x.compute(target_x, latest_chassis_position[0])
        pid_output_y, _, _, _, _, _ = move_pid_y.compute(target_y, latest_chassis_position[1])
        
        # จำกัดความเร็วสูงสุด
        max_speed = 0.8  # m/s
        speed_x = max(-max_speed, min(max_speed, pid_output_x))
        speed_y = max(-max_speed, min(max_speed, pid_output_y))
        
        # สั่งให้หุ่นยนต์เคลื่อนที่ (สไลด์ - ไม่หมุน)
        ep_chassis.drive_speed(x=speed_x, y=speed_y, z=0)
        time.sleep(0.05)
    
    # หยุดการเคลื่อนที่
    ep_chassis.drive_speed(x=0, y=0, z=0)
    
    # คำนวณระยะทางที่เคลื่อนที่ได้จริง
    final_dx = latest_chassis_position[0] - start_pos[0]
    final_dy = latest_chassis_position[1] - start_pos[1]
    final_distance = math.sqrt(final_dx*final_dx + final_dy*final_dy)
    
    print(f"✅ สไลด์เสร็จ {final_distance:.3f}m")
    print(f"📍 ตำแหน่งสุดท้าย: ({latest_chassis_position[0]:.3f}, {latest_chassis_position[1]:.3f})")
    
    time.sleep(0.2)
    return final_distance

def move_direction_pid_with_emergency_brake(ep_chassis, direction, distance):
    """
    เคลื่อนที่ไปยังทิศทางที่กำหนดด้วย PID control (ไม่หมุนตัว - สไลด์)
    มีระบบกันชนฉุกเฉิน: ถ้า filtered_distance < 250mm จะหยุดทันที
    """
    global move_pid_x, move_pid_y, latest_chassis_position
    # from controler.movement_slide import filtered_distance

    direction_map = {
        'x+': {'x': 1, 'y': 0, 'name': 'ข้างหน้า (x+)'},
        'x-': {'x': -1, 'y': 0, 'name': 'ข้างหลัง (x-)'},
        'y+': {'x': 0, 'y': 1, 'name': 'ขวา (y+)'},
        'y-': {'x': 0, 'y': -1, 'name': 'ซ้าย (y-)'}
    }

    if direction not in direction_map:
        print(f"❌ ทิศทางไม่ถูกต้อง: {direction}")
        return 0

    dir_info = direction_map[direction]
    print(f"🚶 สไลด์{dir_info['name']} {distance:.3f}m")

    if move_pid_x is None:
        move_pid_x = PID(kp=1.0, ki=0.1, kd=0)
        
    if move_pid_y is None:
        move_pid_y = PID(kp=1.0, ki=0.1, kd=0)

    move_pid_x.reset()
    move_pid_y.reset()

    start_pos = list(latest_chassis_position)
    target_x = start_pos[0] + (distance * dir_info['x'])
    target_y = start_pos[1] + (distance * dir_info['y'])

    tolerance = 0.05
    stable_iterations = 0
    max_iterations = 150
    iteration = 0

    print(f"📍 เริ่มต้นที่: ({start_pos[0]:.3f}, {start_pos[1]:.3f})")
    print(f"🎯 เป้าหมาย: ({target_x:.3f}, {target_y:.3f})")

    while iteration < max_iterations:
        iteration += 1

        # Emergency brake: ถ้า ToF < 300mm ให้หยุดทันที
        if filtered_distance[0] < 300:
            print("🛑 Emergency Brake! ToF < 250mm")
            ep_chassis.drive_speed(x=0, y=0, z=0)
            ep_chassis.drive_speed(x=0, y=0, z=0)
            ep_chassis.drive_speed(x=0, y=0, z=0)
            ep_chassis.drive_speed(x=0, y=0, z=0)
            ep_chassis.drive_speed(x=0, y=0, z=0)
            ep_chassis.drive_speed(x=0, y=0, z=0)
            ep_chassis.drive_speed(x=0, y=0, z=0)
            ep_chassis.drive_speed(x=0, y=0, z=0)
            break

        dx = latest_chassis_position[0] - start_pos[0]
        dy = latest_chassis_position[1] - start_pos[1]
        current_distance = math.sqrt(dx*dx + dy*dy)

        error_x = target_x - latest_chassis_position[0]
        error_y = target_y - latest_chassis_position[1]
        total_error = math.sqrt(error_x*error_x + error_y*error_y)

        if total_error < tolerance:
            stable_iterations += 1
            if stable_iterations >= 30:
                print("✅ ถึงเป้าหมายแล้ว!")
                break
        else:
            stable_iterations = 0

        pid_output_x, *_ = move_pid_x.compute(target_x, latest_chassis_position[0])
        pid_output_y, *_ = move_pid_y.compute(target_y, latest_chassis_position[1])

        max_speed = 0.8
        speed_x = max(-max_speed, min(max_speed, pid_output_x))
        speed_y = max(-max_speed, min(max_speed, pid_output_y))

        ep_chassis.drive_speed(x=speed_x, y=speed_y, z=0)
        time.sleep(0.05)

    ep_chassis.drive_speed(x=0, y=0, z=0)

    final_dx = latest_chassis_position[0] - start_pos[0]
    final_dy = latest_chassis_position[1] - start_pos[1]
    final_distance = math.sqrt(final_dx*final_dx + final_dy*final_dy)

    print(f"✅ สไลด์เสร็จ {final_distance:.3f}m")
    print(f"📍 ตำแหน่งสุดท้าย: ({latest_chassis_position[0]:.3f}, {latest_chassis_position[1]:.3f})")

    time.sleep(0.2)
    ep_gimbal.moveto(pitch=-6, yaw=0, pitch_speed=200, yaw_speed=200).wait_for_completed()
    return final_distance

def move_direction_pid_wall(ep_chassis, direction, distance):
    """
    เคลื่อนที่ไปยังทิศทางที่กำหนดด้วย PID control สำหรับปรับตำแหน่งกับกำแพง
    
    Args:
        ep_chassis: chassis controller
        direction (str): ทิศทางที่ต้องการเคลื่อนที่ ('x+', 'x-', 'y+', 'y-')
        distance (float): ระยะทางที่ต้องการเคลื่อนที่ (เมตร)
    
    Returns:
        float: ระยะทางที่เคลื่อนที่ได้จริง
    """
    global move_pid_x, move_pid_y, latest_chassis_position
    
    # แมปทิศทางกับการเคลื่อนที่
    direction_map = {
        'x+': {'x': 1, 'y': 0, 'name': 'ข้างหน้า (x+)'},     # ไปข้างหน้า
        'x-': {'x': -1, 'y': 0, 'name': 'ข้างหลัง (x-)'},    # ไปข้างหลัง
        'y+': {'x': 0, 'y': 1, 'name': 'ขวา (y+)'},          # ไปขวา
        'y-': {'x': 0, 'y': -1, 'name': 'ซ้าย (y-)'}         # ไปซ้าย
    }
    
    if direction not in direction_map:
        print(f"❌ ทิศทางไม่ถูกต้อง: {direction}")
        return 0
    
    dir_info = direction_map[direction]
    print(f"🔧 ปรับตำแหน่ง{dir_info['name']} {distance:.3f}m")
    
    # เริ่มต้น PID controllers
    if move_pid_x is None:
        move_pid_x = PID(kp=1.0*0.75, ki=0.1*0.75, kd=0)
    if move_pid_y is None:
        move_pid_y = PID(kp=1.0*0.75, ki=0.1*0.75, kd=0)
    
    # รีเซ็ต PID
    move_pid_x.reset()
    move_pid_y.reset()
    
    # **แก้ไข: ใช้ list() แทน .copy()**
    start_pos = list(latest_chassis_position)
    
    # คำนวณตำแหน่งเป้าหมาย
    target_x = start_pos[0] + (distance * dir_info['x'])
    target_y = start_pos[1] + (distance * dir_info['y'])
    
    tolerance = 0.05  # 5cm สำหรับการปรับตำแหน่ง
    stable_iterations = 0
    max_iterations = 200  # ลดลงเพื่อไม่ให้ใช้เวลานาน
    iteration = 0
    
    while iteration < max_iterations:
        iteration += 1
        
        # คำนวณ error ในแต่ละแกน
        error_x = target_x - latest_chassis_position[0]
        error_y = target_y - latest_chassis_position[1]
        total_error = math.sqrt(error_x*error_x + error_y*error_y)
        
        # ตรวจสอบว่าถึงเป้าหมายแล้วหรือไม่
        if total_error < tolerance:
            stable_iterations += 1
            if stable_iterations >= 45:
                print("✅ ปรับตำแหน่งเสร็จ!")
                break
        else:
            stable_iterations = 0
        
        # คำนวณ PID output สำหรับแต่ละแกน
        pid_output_x, _, _, _, _, _ = move_pid_x.compute(target_x, latest_chassis_position[0])
        pid_output_y, _, _, _, _, _ = move_pid_y.compute(target_y, latest_chassis_position[1])
        
        # จำกัดความเร็วสูงสุด (ช้ากว่าสำหรับการปรับตำแหน่ง)
        max_speed = 0.8  # m/s
        speed_x = max(-max_speed, min(max_speed, pid_output_x))
        speed_y = max(-max_speed, min(max_speed, pid_output_y))
        
        # สั่งให้หุ่นยนต์เคลื่อนที่ (สไลด์ - ไม่หมุน)
        ep_chassis.drive_speed(x=speed_x, y=speed_y, z=0)
        time.sleep(0.05)
    
    # หยุดการเคลื่อนที่
    ep_chassis.drive_speed(x=0, y=0, z=0)
    
    # คำนวณระยะทางที่เคลื่อนที่ได้จริง
    final_dx = latest_chassis_position[0] - start_pos[0]
    final_dy = latest_chassis_position[1] - start_pos[1]
    final_distance = math.sqrt(final_dx*final_dx + final_dy*final_dy)
    
    print(f"✅ ปรับตำแหน่งเสร็จ {final_distance:.3f}m")
    
    time.sleep(0.05)
    return final_distance

def move_forward_pid(ep_chassis, distance):
    """เดินไปข้างหน้า (x+) ด้วย PID"""
    return move_direction_pid(ep_chassis, 'x+', distance)

def move_back_pid(ep_chassis, distance):
    """เดินไปข้างหลัง (x-) ด้วย PID"""
    return move_direction_pid(ep_chassis, 'x-', distance)

def move_right_pid(ep_chassis, distance):
    """เดินไปขวา (y+) ด้วย PID"""
    return move_direction_pid(ep_chassis, 'y+', distance)

def move_left_pid(ep_chassis, distance):
    """เดินไปซ้าย (y-) ด้วย PID"""
    return move_direction_pid(ep_chassis, 'y-', distance)


def move_to_tile_center_from_walls(ep_chassis, way, marker, tof_wall, tile_size=0.6, ep_gimbal=None):
    """
    รับ way, marker, tof_wall (list 4 ช่อง) จาก move_gimbal
    จะคำนวณขอบเขตบล็อก 0.6x0.6m อ้างอิงตำแหน่งปัจจุบัน แล้วเดินไปจุดกึ่งกลางบล็อกนั้น
    ถ้ามีข้อมูลกำแพงเฉพาะบางแกน จะปรับเฉพาะแกนนั้น
    ถ้ามีกำแพงเดียวหรือไม่เจอกำแพงเลย จะใช้ฟังก์ชันหาเสา
    """
    global latest_chassis_position

    # แปลงข้อมูลเป็น dict เฉพาะทิศที่เป็นกำแพง
    wall = {}
    dir_map = ['left', 'front', 'right', 'back']
    for i in range(4):
        if way[i] == 0 and tof_wall[i] is not None:
            wall[dir_map[i]] = tof_wall[i]

    wall_count = len(wall)
    if wall_count > 1:
        # ปกติ: ใช้สูตรเดิม
        x_now, y_now = latest_chassis_position[0], latest_chassis_position[1]
        min_x = x_now - tile_size/2
        max_x = x_now + tile_size/2
        min_y = (y_now - tile_size/2)
        max_y = (y_now + tile_size/2)

        if 'left' in wall:
            min_y = y_now - wall['left']/1000
            max_y = min_y + tile_size
        elif 'right' in wall:
            max_y = y_now + wall['right']/1000
            min_y = max_y - tile_size

        if 'front' in wall:
            max_x = x_now + wall['front']/1000
            min_x = max_x - tile_size
        elif 'back' in wall:
            min_x = x_now - wall['back']/1000
            max_x = min_x + tile_size

        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2

        print(f"🟦 ขอบเขต: x={min_x:.3f}~{max_x:.3f}, y={min_y:.3f}~{max_y:.3f}")
        print(f"🎯 จุดกึ่งกลาง: ({center_x:.3f}, {center_y:.3f})")
        print(f"📍 ตำแหน่งปัจจุบัน: ({x_now:.3f}, {y_now:.3f})")
        print(f"🔖 marker: {marker}")

        # เดินไปจุดกึ่งกลาง เฉพาะแกนที่มีข้อมูล
        if ('front' in wall or 'back' in wall):
            move_direction_pid(ep_chassis, 'x+', center_x - x_now)
            correct_robot_orientation( target_yaw=0)
        if ('left' in wall or 'right' in wall):
            move_direction_pid(ep_chassis, 'y+', center_y - y_now)
            correct_robot_orientation( target_yaw=0)
    else:
        # มีกำแพงเดียวหรือไม่เจอกำแพงเลย: หาเสา
        wall_side = None
        for k in wall.keys():
            wall_side = k
        if ep_gimbal is not None:
            print("🔎 มีกำแพงเดียวหรือไม่เจอกำแพงเลย กำลังหาเสาเพื่อคำนวณศูนย์กลาง...")
            position = find_pillar_and_move_to_center(ep_chassis, ep_gimbal, way, tof_wall, tile_size)
            center_x = position[0]
            center_y = position[1]

            # เพิ่มบรรทัดนี้!
            x_now, y_now = latest_chassis_position[0], latest_chassis_position[1]

            # correct_robot_orientation( target_yaw=0)
            move_to_center(ep_chassis, center_x, center_y, get_position_func=lambda: (latest_chassis_position[0], latest_chassis_position[1]))
            
            # move_direction_pid(ep_chassis, 'y+', center_y - y_now)
            correct_robot_orientation( target_yaw=0)

        else:
            print("❌ ไม่สามารถหาเสาได้ (ไม่ได้ส่ง ep_gimbal มา)")
def sweep_angles_list(start, end, step):
    """สร้างลิสต์มุม sweep ที่รองรับทั้งกรณี start < end และ start > end"""
    if start < end:
        return list(range(start, end + 1, step))
    else:
        return list(range(start, end - 1, -step))

import math
import time

# สมมติว่ามีฟังก์ชันเหล่านี้อยู่แล้ว
# def get_stable_distance_reading():
#     # คืนค่าระยะทางเป็น mm
#     return 500
#
# def sweep_angles_list(start, end, step):
#     # สร้าง list ของมุมสำหรับการ sweep
#     return list(range(start, end + step, step))

def find_pillar_and_move_to_center(ep_chassis, ep_gimbal, way, tof_wall, tile_size=0.6):
    """
    ค้นหาเสา คำนวณขอบเขตของ Tile และหาจุดศูนย์กลาง
    way: [ซ้าย, หน้า, ขวา, หลัง] (0=มีกำแพง, 1=ไม่มี)
    tof_wall: ระยะห่างจากกำแพง [ซ้าย, หน้า, ขวา, หลัง] (mm)
    คืนค่า: tuple (center_x, center_y) หรือ None ถ้าข้อมูลไม่น่าเชื่อถือ
    """
    global latest_chassis_position
    if latest_chassis_position is None or len(latest_chassis_position) < 2:
        print("Error: latest_chassis_position is not available.")
        return None

    x_now, y_now = latest_chassis_position[0], latest_chassis_position[1]
    print(f"🤖 Robot starting at: ({x_now:.3f}, {y_now:.3f})")

    # --- 1. กำหนดมุมกวาดที่ถูกต้องและชัดเจน ---
    PILLAR_NAMES = ["FORWARD_LEFT (+x, -y)", "FORWARD_RIGHT (+x, +y)", "BACK_LEFT (-x, -y)", "BACK_RIGHT (-x, +y)"]
    SWEEP_RANGES = [
        (-80, -10, 10),    # FORWARD_LEFT -> ศูนย์กลางที่ -45
        (10, 80, 10),      # FORWARD_RIGHT -> ศูนย์กลางที่ +45
        (-170, -100, 10),  # BACK_LEFT -> ศูนย์กลางที่ -135
        (100, 170, 10),    # BACK_RIGHT -> ศูนย์กลางที่ +135
    ]

    skip_sweep = [False] * 4
    if way[1] == 0: skip_sweep[0] = True; skip_sweep[1] = True  # กำแพงหน้า
    if way[3] == 0: skip_sweep[2] = True; skip_sweep[3] = True  # กำแพงหลัง
    if way[0] == 0: skip_sweep[0] = True; skip_sweep[2] = True  # กำแพงซ้าย
    if way[2] == 0: skip_sweep[1] = True; skip_sweep[3] = True  # กำแพงขวา

    # --- 2. กวาดหาพิกัดของเสาทุกต้น ---
    pillar_coords = [None] * 4
    for i in range(4):
        if skip_sweep[i]:
            print(f"🚧 Skipping {PILLAR_NAMES[i]} due to wall.")
            continue

        print(f"🔎 Sweeping for {PILLAR_NAMES[i]}...")
        readings = []
        sweep_angles = sweep_angles_list(*SWEEP_RANGES[i])
        for yaw in sweep_angles:
            ep_gimbal.moveto(pitch=-6, yaw=yaw, pitch_speed=50, yaw_speed=50).wait_for_completed()
            dist_m = get_stable_distance_reading() / 1000.0
            readings.append((dist_m, yaw))

        valid_readings = [r for r in readings if 0.1 < r[0] < tile_size * 1.5]
        if valid_readings:
            min_dist, min_yaw = min(valid_readings, key=lambda x: x[0])
            rad = math.radians(min_yaw)
            px = x_now + min_dist * math.cos(rad)
            py = y_now + min_dist * math.sin(rad)
            pillar_coords[i] = (px, py)
            print(f"  📌 Found {PILLAR_NAMES[i]} at ({px:.3f}, {py:.3f})")
        else:
            print(f"  ❌ Did not find {PILLAR_NAMES[i]}.")

    # --- 3. ตรวจสอบความน่าเชื่อถือ (Sanity Check) ---
    found_pillars_count = sum(1 for p in pillar_coords if p is not None)
    print(f"✅ Found {found_pillars_count} pillars in total.")

    MIN_PILLARS_REQUIRED = 2
    if way == [1, 1, 1, 1] and found_pillars_count < MIN_PILLARS_REQUIRED:
        print(f"⚠️ No walls and not enough pillars found ({found_pillars_count} < {MIN_PILLARS_REQUIRED}). Aborting.")
        return None

    # --- 4. คำนวณหาขอบเขต (min/max) ---
    min_x, max_x, min_y, max_y = None, None, None, None

    if way[1] == 0 and tof_wall[1] is not None: max_x = x_now + tof_wall[1] / 1000.0
    if way[3] == 0 and tof_wall[3] is not None: min_x = x_now - tof_wall[3] / 1000.0
    if way[0] == 0 and tof_wall[0] is not None: min_y = y_now - tof_wall[0] / 1000.0
    if way[2] == 0 and tof_wall[2] is not None: max_y = y_now + tof_wall[2] / 1000.0

    front_pillars_x = [p[0] for i, p in enumerate(pillar_coords) if i in [0, 1] and p is not None]
    back_pillars_x = [p[0] for i, p in enumerate(pillar_coords) if i in [2, 3] and p is not None]
    left_pillars_y = [p[1] for i, p in enumerate(pillar_coords) if i in [0, 2] and p is not None]
    right_pillars_y = [p[1] for i, p in enumerate(pillar_coords) if i in [1, 3] and p is not None]

    if max_x is None and front_pillars_x: max_x = max(front_pillars_x)
    if min_x is None and back_pillars_x:   min_x = min(back_pillars_x)
    if min_y is None and left_pillars_y:   min_y = min(left_pillars_y)
    if max_y is None and right_pillars_y:   max_y = max(right_pillars_y)

    # --- 5. ใช้ tile_size เป็นค่าสำรอง ---
    if max_x is not None and min_x is None: min_x = max_x - tile_size
    if min_x is not None and max_x is None: max_x = min_x + tile_size
    if max_y is not None and min_y is None: min_y = max_y - tile_size
    if min_y is not None and max_y is None: max_y = min_y + tile_size

    if all(v is None for v in [min_x, max_x, min_y, max_y]):
        print("⚠️ Cannot determine any boundary. Aborting.")
        return None

    # --- 6. คำนวณหาจุดศูนย์กลาง ---
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2

    print(f"🟦 Calculated Box: x=[{min_x:.3f}, {max_x:.3f}], y=[{min_y:.3f}, {max_y:.3f}]")
    print(f"🎯 Target Center: ({center_x:.3f}, {center_y:.3f})")

    return (center_x, center_y)




def move_to_center(ep_chassis, target_x, target_y, get_position_func, tolerance=0.05, max_time=5.0):
    """
    เคลื่อนที่ไปยัง (target_x, target_y) ทีเดียวด้วย PID 2 แกน
    get_position_func: ฟังก์ชันคืนค่าตำแหน่งปัจจุบัน (x, y)
    """
    global move_pid_x, move_pid_y

    # สร้าง PID ถ้ายังไม่มี
    if move_pid_x is None:
        move_pid_x = PID(kp=1.0, ki=0.1, kd=0)
    if move_pid_y is None:
        move_pid_y = PID(kp=1.0, ki=0.1, kd=0)

    move_pid_x.reset()
    move_pid_y.reset()

    start_time = time.time()
    stable_count = 0

    while True:
        x_now, y_now = latest_chassis_position[0], latest_chassis_position[1]
        error_x = target_x - x_now
        error_y = target_y - y_now
        dist = math.hypot(error_x, error_y)

        # คำนวณ PID output
        pid_output_x, *_ = move_pid_x.compute(target_x, x_now)
        pid_output_y, *_ = move_pid_y.compute(target_y, y_now)

        # จำกัดความเร็ว
        max_speed = 0.8
        speed_x = max(-max_speed, min(max_speed, pid_output_x))
        speed_y = max(-max_speed, min(max_speed, pid_output_y))

        ep_chassis.drive_speed(x=speed_x, y=speed_y, z=0)

        if dist < tolerance:
            stable_count += 1
            if stable_count > 50:
                break
        else:
            stable_count = 0

        if time.time() - start_time > max_time:
            break
        time.sleep(0.05)
    # correct_robot_orientation( target_yaw=0)
    ep_chassis.drive_speed(x=0, y=0, z=0)
    ep_gimbal.moveto(pitch=-6, yaw=0, pitch_speed=500, yaw_speed=500).wait_for_completed()
    correct_robot_orientation( target_yaw=0)



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







# --- ฟังก์ชัน Helper ---
def format_coords(coords):
    """จัดรูปแบบพิกัดทศนิยมเพื่อการแสดงผล"""
    return f"({coords[0]:.2f}, {coords[1]:.2f})"


def get_stable_distance_reading():
    """
    อ่านค่าระยะทางแบบ stable โดยรอให้ค่าคงที่
    
    Returns:
        float: ค่าระยะทางที่ stable
    """
    stable_readings = []

    # อ่านค่า 11 ครั้ง
    for i in range(5):
        stable_readings.append(lastest_distance[0])
        time.sleep(0.03)  # รอ 30ms

    # ใช้ median ของ 5 ค่า
    stable_value = statistics.median(stable_readings)
    
    # ตรวจสอบความเสถียร
    variance = max(stable_readings) - min(stable_readings)
    if variance > 100:  # ถ้าค่าผันแปรมากกว่า 10cm
        print(f"⚠️  ToF ไม่เสถียร: variance={variance:.0f}mm, readings={stable_readings}")
    
    return stable_value


# def detect_red_color(ep_camera):
#     time.sleep(0.5)
#     """
#     ตรวจจับสีแดงจากภาพที่ได้จากกล้อง Robomaster
#     Args:
#         ep_camera (robomaster.camera): camera module ของ Robomaster
#     Returns:
#         bool: True หากพบสีแดง, False หากไม่พบ
#     """
#     img = ep_camera.read_cv2_image()
#     if img is None:
#         print("❌ ไม่สามารถอ่านภาพจากกล้องได้")
#         return False

#     # ลด noise
#     img_blur = cv2.GaussianBlur(img, (5, 5), 0)

#     # แปลงเป็น HSV
#     hsv_img = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)

#     # ช่วงสีแดงแบบกว้างขึ้น
#     lower_red1 = np.array([0, 80, 80])    # Hue ต่ำ, เพิ่ม S/V ให้สูงขึ้น
#     upper_red1 = np.array([10, 255, 255])

#     lower_red2 = np.array([170, 80, 80])  # Hue สูง
#     upper_red2 = np.array([180, 255, 255])

#     mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
#     mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)

#     final_mask = mask1 + mask2

#     # เปิด/ปิดช่องว่างเล็กๆ ใน mask
#     kernel = np.ones((5, 5), np.uint8)
#     final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, kernel)
#     final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel)

#     # ตรวจจับ pixel สีแดง
#     red_pixels = np.count_nonzero(final_mask)
#     if red_pixels > 5000:  # ปรับ threshold ตามความเหมาะสม
#         print(f"✅ พบสีแดง ({red_pixels} pixels)")
#         return True
#     else:
#         print(f"❌ ไม่พบสีแดง ({red_pixels} pixels)")
#         return False



# --- ฟังก์ชันสแกนที่ปรับปรุงด้วย Median Filter ---
def move_gimbal(ep_gimbal, ep_chassis, ep_vision, ep_camera):
    """
    ฟังก์ชันสแกน 4 ทิศทาง + Median Filter + ปรับทิศทางให้ตรง
    และขยับไปจุดกึ่งกลางบล็อก (รองรับกรณีเจอกำแพงเดียวหรือไม่เจอกำแพงเลย)
    """
    # from controler.movement_slide import latest_chassis_position, move_to_center, move_to_tile_center_from_walls

    way = [0, 0, 0, 0]  # [ซ้าย, หน้า, ขวา, หลัง]
    marker = ["No", "No", "No", "No"]  # [ซ้าย, หน้า, ขวา, หลัง]
    stable_distances = [None, None, None, None]

    print("🧭 ตรวจสอบและปรับทิศทางก่อนสแกน...")
    correct_robot_orientation( target_yaw=0)
    time.sleep(0.1)

    # sweep 4 ทิศ เก็บค่าก่อน
    yaws = [-90, 0, 90, 180]
    for i, yaw in enumerate(yaws):
        ep_gimbal.moveto(pitch=-6, yaw=yaw, pitch_speed=25, yaw_speed=25).wait_for_completed()
        time.sleep(0.1)
        stable_distance = get_stable_distance_reading()
        stable_distances[i] = stable_distance
        print(f"หัน {['ซ้าย','หน้า','ขวา','หลัง'][i]}: raw={lastest_distance[0]:.0f}mm, stable={stable_distance:.0f}mm")
        # ตรวจจับ marker หลังสแกนแต่ละทิศ (ถ้าต้องการ)
        # if detect_red_color(ep_camera):
        #     checkmarkers(i, marker, ep_vision, yaw)

    # ประมวลผลผลลัพธ์หลังสแกน
    for i, stable_distance in enumerate(stable_distances):
        if i == 3:
            wall_th = 200
        else:
            wall_th = 170
        if stable_distance < 200:
            way[i] = 0
        elif stable_distance < 400:
            way[i] = 0
        else:
            way[i] = 1

    # === ปรับตำแหน่งกับกำแพงทีเดียวหลังสแกน (ขยับ x,y พร้อมกัน) ===
    x_now, y_now = latest_chassis_position[0], latest_chassis_position[1]
    tile_size_x = 0.6   # หน้า-หลัง
    tile_size_y = 0.6  # ซ้าย-ขวา (170mm)

    wall_count = way.count(0)
    if wall_count < 2:
        # ถ้ามีกำแพงเดียวหรือไม่เจอกำแพงเลย ให้ใช้ฟังก์ชันนี้
        print("🔎 เจอกำแพงเดียวหรือไม่เจอกำแพงเลย ใช้ move_to_tile_center_from_walls")
        move_to_tile_center_from_walls(
            ep_chassis,
            way,
            marker,
            stable_distances,
            tile_size=0.6,
            ep_gimbal=ep_gimbal
        )
        
    else:
        # เดิม: ขยับ x,y พร้อมกันด้วย PID
        # --- Y (ซ้าย-ขวา) ---
        tile_size_x = 0.6-0.2   # หน้า-หลัง
        tile_size_y = 0.6-0.2  # ซ้าย-ขวา (170mm)
        if way[0] == 0 and way[2] == 0:
            min_y = (y_now - stable_distances[0]/1000)
            max_y = (y_now + stable_distances[2]/1000)
            center_y = ((min_y + max_y) / 2)
        elif way[0] == 0:
            min_y = (y_now - stable_distances[0]/1000)
            center_y = (min_y + tile_size_y/2)
        elif way[2] == 0:
            max_y = (y_now + stable_distances[2]/1000)
            center_y = (max_y - tile_size_y/2)
        else:
            center_y = y_now

        # --- X (หน้า-หลัง) ---
        if way[1] == 0 and way[3] == 0:
            min_x = x_now - stable_distances[3]/1000
            max_x = x_now + stable_distances[1]/1000
            center_x = (min_x + max_x) / 2
        elif way[1] == 0:
            max_x = x_now + stable_distances[1]/1000
            center_x = max_x - tile_size_x/2
        elif way[3] == 0:
            min_x = x_now - stable_distances[3]/1000
            center_x = min_x + tile_size_x/2
        else:
            center_x = x_now

        print(f"🎯 ขยับทีเดียวไปจุดกึ่งกลาง tile: ({center_x:.3f}, {center_y:.3f})")

        move_to_center(
            ep_chassis,
            center_x,
            center_y,
            get_position_func=lambda: (latest_chassis_position[0], latest_chassis_position[1])
        )
    # correct_robot_orientation( target_yaw=0)

    # หันกลับด้านหน้าและปรับทิศทางสุดท้าย
    ep_gimbal.moveto(pitch=-6, yaw=0, pitch_speed=300, yaw_speed=300).wait_for_completed()
    time.sleep(0.1)
    print("🧭 ปรับทิศทางสุดท้ายหลังสแกน...")
    correct_robot_orientation( target_yaw=0)

    print(f"📊 ผลการสแกนหลังกรอง: {way} [ซ้าย, หน้า, ขวา, หลัง]")
    print(f"📍 ผลการสแกน Marker: {marker} [ซ้าย, หน้า, ขวา, หลัง]")

    return way, marker  # คืนค่า way และ marker

# --- ฟังก์ชันหลักในการสำรวจด้วย DFS ---
def explore_from(current_coords, ep_chassis, ep_gimbal, ep_vision, ep_camera):
    """
    ฟังก์ชัน DFS หลัก (เวอร์ชันแก้ไขสมบูรณ์)
    - สำรวจแบบ Recursive
    - ตรวจสอบขอบเขตแผนที่ 7x7 ภายในตัว
    - แก้ไขปัญหาการอ่านค่า STEP_SIZE
    """
    global visited_nodes, scan_memory, maze_graph, coord_to_node_id

    # =======================================================================
    # ส่วนที่ 1: กำหนดขอบเขตแผนที่และฟังก์ชันตรวจสอบ (ทั้งหมดอยู่ภายในนี้)
    # =======================================================================
    MAP_SIZE = 7
    STEP_SIZE = 0.6  # **กำหนดค่าที่ถูกต้องภายในฟังก์ชันโดยตรง**

    # คำนวณขอบเขตของแผนที่ โดยมีจุด (0,0) เป็นศูนย์กลาง
    # สำหรับแผนที่ 7x7 จะมี 3 ก้าวจากจุดศูนย์กลางไปในแต่ละทิศ
    # ดังนั้น ขอบเขตคือ: 3 * 0.6 = 2.1 เมตร
    MAP_MIN_X = 0.0
    MAP_MIN_Y = 0.0
    MAP_MAX_X = (MAP_SIZE - 1) * STEP_SIZE  # ผลลัพธ์ที่ถูกต้องคือ 0.6
    MAP_MAX_Y = (MAP_SIZE - 1) * STEP_SIZE  # ผลลัพธ์ที่ถูกต้องคือ 0.6

    # สร้างฟังก์ชันซ้อน (Nested Function) สำหรับตรวจสอบขอบเขต
    def is_within_bounds(coords):
        """ตรวจสอบว่าพิกัด (x, y) อยู่ในขอบเขตที่คำนวณไว้หรือไม่"""
        x, y = coords
        epsilon = 0.01  # ค่าเผื่อสำหรับความคลาดเคลื่อนของตัวเลขทศนิยม
        
        # คืนค่า True หากพิกัดอยู่ในช่วง Min ถึง Max ทั้งแกน X และ Y
        return (MAP_MIN_X - epsilon <= x <= MAP_MAX_X + epsilon and
                MAP_MIN_Y - epsilon <= y <= MAP_MAX_Y + epsilon)

    # ตรวจสอบพิกัดเริ่มต้น (จะทำงานแค่ครั้งแรกที่เรียกฟังก์ชัน)
    if not visited_nodes and not is_within_bounds(current_coords):
        print(f"❌ ข้อผิดพลาด: จุดเริ่มต้น {format_coords(current_coords)} อยู่นอกขอบเขตแผนที่!")
        raise ValueError("Start coordinate is out of bounds.")
    # =======================================================================
    # จบส่วนกำหนดขอบเขต
    # =======================================================================

    # --- ส่วนตรรกะการสำรวจ (เหมือนเดิม แต่ตรวจสอบให้แน่ใจว่าถูกต้อง) ---

    visited_nodes.add(current_coords)
    key = (round(current_coords[0], 2), round(current_coords[1], 2))
    if key not in coord_to_node_id:
        node_id = maze_graph.add_node(key[0], key[1])
        coord_to_node_id[key] = node_id
    else:
        node_id = coord_to_node_id[key]
        print(f"🔍 พบโหนดเดิม: ID={node_id} ที่ {format_coords(key)}")

    if current_coords in scan_memory:
        available_ways = scan_memory[current_coords]
        print(f"\n🧠 กลับมาที่ {format_coords(current_coords)}, ใช้ข้อมูลสแกนเก่า: {available_ways}")
        if not maze_graph.nodes[node_id].blocked_directions:
            maze_graph.add_blocked_direction_to_node(node_id, available_ways)
    else:
        print(f"\n📍 มาถึงพิกัดใหม่: {format_coords(current_coords)}. เริ่มสแกน...")
        tof_readings.clear()
        time.sleep(0.2)
        # correct_robot_orientation( target_yaw=0)
        available_ways, marker = move_gimbal(ep_gimbal, ep_chassis, ep_vision, ep_camera)
        # correct_robot_orientation( target_yaw=0)
        scan_memory[current_coords] = available_ways
        print(f"🔬 ผลการสแกนถูกบันทึก: {available_ways}")
        maze_graph.add_blocked_direction_to_node(node_id, available_ways)
        maze_graph.add_marker_direction_to_node(node_id, marker)

    direction_map = [
        {'name': 'left',     'direction': 'y-', 'coords': (0, -STEP_SIZE), 'gimbal_yaw': -90},
        {'name': 'forward',  'direction': 'x+', 'coords': (STEP_SIZE, 0), 'gimbal_yaw': 0},
        {'name': 'right',    'direction': 'y+', 'coords': (0, STEP_SIZE), 'gimbal_yaw': 90},
        {'name': 'backward', 'direction': 'x-', 'coords': (-STEP_SIZE, 0), 'gimbal_yaw': 180},
    ]

    # --- ส่วนลูปการสำรวจ (จุดสำคัญที่ทำการตรวจสอบ) ---
    for i, direction_info in enumerate(direction_map):
        # 1. ตรวจสอบว่าผลสแกนบอกว่าทางเปิดหรือไม่
        if available_ways[i] == 1:
            d_coords = direction_info['coords']
            next_coords = (round(current_coords[0] + d_coords[0], 2), round(current_coords[1] + d_coords[1], 2))

            # 2. **ตรวจสอบว่าพิกัดถัดไป "อยู่นอกขอบเขต" หรือไม่**
            if not is_within_bounds(next_coords):
                print(f"  -- 🗺️  เส้นทาง {direction_info['name']} ไปยัง {format_coords(next_coords)} อยู่นอกขอบเขต! (ถูกบล็อก)")
                continue  # ข้ามไปตรวจสอบทิศทางถัดไป ไม่ทำอะไรต่อ

            # 3. **ตรวจสอบว่า "เคยไปแล้ว" หรือไม่**
            if next_coords in visited_nodes:
                print(f"  -- 📌 เส้นทาง {direction_info['name']} ไปยัง {format_coords(next_coords)} เคยไปแล้ว ข้ามไป...")
                # (ส่วนเชื่อมเส้นทางที่เคยไปแล้ว)
                next_key = (round(next_coords[0], 2), round(next_coords[1], 2))
                if next_key in coord_to_node_id:
                    current_node_id = coord_to_node_id[key]
                    next_node_id = coord_to_node_id[next_key]
                    if next_node_id not in maze_graph.nodes[current_node_id].connections:
                        distance = math.sqrt((next_coords[0] - current_coords[0])**2 + (next_coords[1] - current_coords[1])**2)
                        maze_graph.add_edge(current_node_id, next_node_id, distance)
                continue # ข้ามไปตรวจสอบทิศทางถัดไป

            # 4. **ถ้าผ่านทุกเงื่อนไข: เริ่มเคลื่อนที่ไปยังพิกัดใหม่**
            print(f"  -> 👣 สไลด์ {direction_info['name']} ไปยัง {format_coords(next_coords)}...")

            # (ส่วนที่เหลือของโค้ดในการเคลื่อนที่และเรียกซ้ำ เหมือนเดิม)
            next_key = (round(next_coords[0], 2), round(next_coords[1], 2))
            if next_key not in coord_to_node_id:
                next_node_id = maze_graph.add_node(next_key[0], next_key[1])
                coord_to_node_id[next_key] = next_node_id
            else:
                next_node_id = coord_to_node_id[next_key]

            current_node_id = coord_to_node_id[key]
            if next_node_id not in maze_graph.nodes[current_node_id].connections:
                distance = math.sqrt((next_coords[0] - current_coords[0])**2 + (next_coords[1] - current_coords[1])**2)
                maze_graph.add_edge(current_node_id, next_node_id, distance)

            gimbal_yaw = direction_info['gimbal_yaw']
            ep_gimbal.moveto(pitch=-6, yaw=gimbal_yaw, pitch_speed=200, yaw_speed=200).wait_for_completed()
            time.sleep(0.1)

            # correct_robot_orientation( target_yaw=0)
            move_direction_pid_with_emergency_brake(ep_chassis, direction_info['direction'], STEP_SIZE)
            
            correct_robot_orientation( target_yaw=0)

            explore_from(next_coords, ep_chassis, ep_gimbal, ep_vision, ep_camera)

            print(f"  <- ⏪ สไลด์กลับจาก {format_coords(next_coords)} มายัง {format_coords(current_coords)}...")

            reverse_direction_map = {'x+': 'x-', 'x-': 'x+', 'y+': 'y-', 'y-': 'y+'}
            reverse_gimbal_map = {'x+': 180, 'x-': 0, 'y+': -90, 'y-': 90}
            reverse_direction = reverse_direction_map[direction_info['direction']]
            reverse_gimbal_yaw = reverse_gimbal_map[direction_info['direction']]

            ep_gimbal.moveto(pitch=-6, yaw=reverse_gimbal_yaw, pitch_speed=200, yaw_speed=200).wait_for_completed()
            time.sleep(0.1)

            # correct_robot_orientation( target_yaw=0)
            move_direction_pid_with_emergency_brake(ep_chassis, reverse_direction, STEP_SIZE)
            # correct_robot_orientation( target_yaw=0)
            available_ways2, marker = move_gimbal(ep_gimbal, ep_chassis, ep_vision, ep_camera)
            # correct_robot_orientation( target_yaw=0)

            
            time.sleep(0.1)

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
    ep_chassis.sub_attitude(freq=50, callback=sub_chassis_attitude)
    ep_gimbal.sub_angle(freq=50, callback=sub_data_angle)
    # ep_camera.start_video_stream(display=True, resolution=camera.STREAM_360P)  # **เริ่มสตรีมวิดีโอจากกล้อง**
    ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
    
    time.sleep(0.5)  # **เพิ่มเวลารอให้ median filter buffer เติม**

    print(f"🔧 Median Filter เริ่มต้นแล้ว (buffer size: {tof_readings.maxlen})")
    print(f"🗺️  Map System เริ่มต้นแล้ว")

    try:
        start_node = (0, 0)
        explore_from(start_node, ep_chassis, ep_gimbal, ep_vision, ep_camera)

        
        # **แสดงสรุปแมพหลังสำรวจเสร็จ**
        print("\n" + "="*60)
        print("🗺️  สรุปแมพที่สร้างขึ้น")
        print("="*60)
        print(f"📊 จำนวนโหนดทั้งหมด: {len(maze_graph.nodes)}")
        print(f"📊 จำนวนเส้นทางทั้งหมด: {sum(len(node.connections) for node in maze_graph.nodes.values()) // 2}")
        
        print("\n📍 รายละเอียดโหนด:")
        import csv

        with open("maze_nodes.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["node_id", "x", "y", "blocked_directions"])
            for node_id, node in maze_graph.nodes.items():
                blocked = node.blocked_directions
                marker = node.marker_directions
                print(f"   โหนด {node_id}: ({node.x:.2f}, {node.y:.2f})")
                print(f"     - ข้อมูลสแกน: {blocked}")  # [y-, x+, y+, x-]
                print(f"     - ข้อมูลสแกน marker: {marker}")
                writer.writerow([
                    node_id,
                    f"{node.x:.3f}",
                    f"{node.y:.3f}",
                    repr(blocked)
                ])
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n===== ⏹️ ถูกอินเตอร์รัพท์: บันทึกแมพก่อนปิดระบบ =====")
        with open("maze_nodes.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["node_id", "x", "y", "blocked_directions"])
            for node_id, node in maze_graph.nodes.items():
                blocked = node.blocked_directions
                marker = node.marker_directions
                print(f"   โหนด {node_id}: ({node.x:.2f}, {node.y:.2f})")
                print(f"     - ข้อมูลสแกน: {blocked}")  # [y-, x+, y+, x-]
                print(f"     - ข้อมูลสแกน marker: {marker}")
                writer.writerow([
                    node_id,
                    f"{node.x:.3f}",
                    f"{node.y:.3f}",
                    repr(blocked)
                ])
    finally:
        print("\n===== ⏹️ การสำรวจเสร็จสิ้น ทำการปิดระบบ =====")
        ep_sensor.unsub_distance()
        ep_chassis.unsub_position()
        ep_chassis.unsub_attitude()
        ep_vision.unsub_detect_info(name="marker")
        
        # ep_camera.stop_video_stream()
        time.sleep(1)
        ep_robot.close()
        with open("maze_nodes.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["node_id", "x", "y", "blocked_directions"])
            for node_id, node in maze_graph.nodes.items():
                blocked = node.blocked_directions
                marker = node.marker_directions
                print(f"   โหนด {node_id}: ({node.x:.2f}, {node.y:.2f})")
                print(f"     - ข้อมูลสแกน: {blocked}")  # [y-, x+, y+, x-]
                print(f"     - ข้อมูลสแกน marker: {marker}")
                writer.writerow([
                    node_id,
                    f"{node.x:.3f}",
                    f"{node.y:.3f}",
                    repr(blocked)
                ])