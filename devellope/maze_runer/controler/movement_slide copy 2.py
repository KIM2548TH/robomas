# -*-coding:utf-8-*-
import csv
import os
import time
import statistics
import math
from collections import deque

from .pid import PID, TurnPID
from logger import log_robot_data

# --- Global Variables ---
latest_chassis_position = [0, 0, 0]
latest_chassis_attitude = [0, 0, 0]
lastest_distance = [0]
tof_readings = deque(maxlen=5)   # <-- แก้จาก list เป็น deque
filtered_distance = [0]
markers = []

LOG_CSV_PATH = "robot_log.csv"
log_csv_header_written = False

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

# ตัวแปร global สำหรับ PID และ tracking
move_pid_x = None
move_pid_y = None
turn_pid = None  # **เพิ่ม: PID สำหรับการหมุน**

def correct_robot_orientation(ep_chassis, target_yaw=0):
    """
    ปรับให้หุ่นยนต์หันตรงตามเป้าหมาย
    
    Args:
        ep_chassis: chassis controller
        target_yaw (float): มุม yaw เป้าหมาย (องศา)
    """
    global turn_pid, latest_chassis_attitude
    
    # เริ่มต้น PID สำหรับการหมุน
    if turn_pid is None:
        turn_pid = TurnPID(kp=2.0*0.40, ki=0.007*0.40, kd=0.03*0.40)
    
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
    if abs(yaw_error) < 0.1:
        print("✅ ทิศทางถูกต้องแล้ว")
        return
    
    tolerance = 0.1  # ความคลาดเคลื่อนที่ยอมรับได้
    stable_count = 0
    max_iterations = 50
    iteration = 0
    
    while iteration < max_iterations:
        iteration += 1
        
        current_yaw = latest_chassis_attitude[0]
        
        # คำนวณ PID output
        turn_output, angle_error, p, i, d = turn_pid.compute(target_yaw, current_yaw)
        
        # จำกัดความเร็วการหมุน
        max_turn_speed = 200  # องศา/วินาที
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
        move_pid_x = PID(kp=2.0, ki=0.01, kd=0)
    if move_pid_y is None:
        move_pid_y = PID(kp=2.0, ki=0.01, kd=0)
    
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
    max_iterations = 50
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
    from controler.movement_slide import filtered_distance

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
        move_pid_x = PID(kp=2.0, ki=0.01, kd=0)
    if move_pid_y is None:
        move_pid_y = PID(kp=2.0, ki=0.01, kd=0)

    move_pid_x.reset()
    move_pid_y.reset()

    start_pos = list(latest_chassis_position)
    target_x = start_pos[0] + (distance * dir_info['x'])
    target_y = start_pos[1] + (distance * dir_info['y'])

    tolerance = 0.05
    stable_iterations = 0
    max_iterations = 50
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
        move_pid_x = PID(kp=2.0*0.75, ki=0.01*0.75, kd=0.03*0.75)
    if move_pid_y is None:
        move_pid_y = PID(kp=2.0*0.75, ki=0.01*0.75, kd=0.03*0.75)
    
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
    if wall_count >= 2:
        # ปกติ: ใช้สูตรเดิม
        x_now, y_now = latest_chassis_position[0], latest_chassis_position[1]
        min_x = x_now - tile_size/2
        max_x = x_now + tile_size/2
        min_y = (y_now - tile_size/2)
        max_y = (y_now + tile_size/2)

        if 'left' in wall:
            min_y = y_now - wall['left']#/1000
            max_y = min_y + tile_size
        elif 'right' in wall:
            max_y = y_now + wall['right']#/1000
            min_y = max_y - tile_size

        if 'front' in wall:
            max_x = x_now + wall['front']#/1000
            min_x = max_x - tile_size
        elif 'back' in wall:
            min_x = x_now - wall['back']#/1000
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
            correct_robot_orientation(ep_chassis, target_yaw=0)
        if ('left' in wall or 'right' in wall):
            move_direction_pid(ep_chassis, 'y+', center_y - y_now)
            correct_robot_orientation(ep_chassis, target_yaw=0)
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

            correct_robot_orientation(ep_chassis, target_yaw=0)
            move_to_center(ep_chassis, center_x, center_y, get_position_func=lambda: (latest_chassis_position[0], latest_chassis_position[1]))
            # move_direction_pid(ep_chassis, 'y+', center_y - y_now)
            correct_robot_orientation(ep_chassis, target_yaw=0)

        else:
            print("❌ ไม่สามารถหาเสาได้ (ไม่ได้ส่ง ep_gimbal มา)")
def sweep_angles_list(start, end, step):
    """สร้างลิสต์มุม sweep ที่รองรับทั้งกรณี start < end และ start > end"""
    if start < end:
        return list(range(start, end + 1, step))
    else:
        return list(range(start, end - 1, -step))

def find_pillar_and_move_to_center(ep_chassis, ep_gimbal, way, tof_wall, tile_size=0.6):
    global latest_chassis_position

    if latest_chassis_position is None or len(latest_chassis_position) < 2:
        print(latest_chassis_position)
        raise ValueError("latest_chassis_position ยังไม่มีข้อมูล")

    x_now, y_now = latest_chassis_position[0], latest_chassis_position[1]

    # [ซ้ายบน, ขวาบน, ซ้ายล่าง, ขวาล่าง] (กวาดช่วงละ 90 องศา)
    sweep_map = [
        sweep_angles_list(-135-15, -45-15, 5),  # ซ้ายบน
        sweep_angles_list(45-15, 135-15, 5),    # ขวาบน
        sweep_angles_list(-135-15, -45-15, 5),  # ซ้ายล่าง
        sweep_angles_list(45-15, 135-15, 5),    # ขวาบนล่าง
    ]

    pillar_yaws = [None, None, None, None]
    pillar_ds = [None, None, None, None]

    skip = [False, False, False, False]
    if way[0] == 0:  # มีกำแพงซ้าย
        skip[0] = True
        skip[2] = True
    if way[2] == 0:  # มีกำแพงขวา
        skip[1] = True
        skip[3] = True
    if way[1] == 0:  # มีกำแพงหน้า
        skip[0] = True
        skip[1] = True
    if way[3] == 0:  # มีกำแพงหลัง
        skip[2] = True
        skip[3] = True

    for i in range(4):
        if skip[i]:
            print(f"🚧 ข้าม sweep {['ซ้ายบน','ขวาบน','ซ้ายล่าง','ขวาล่าง'][i]} เพราะมี/ติดกำแพง")
            continue
        readings = []
        for yaw in sweep_map[i]:
            ep_gimbal.moveto(pitch=-6, yaw=yaw, pitch_speed=350, yaw_speed=350).wait_for_completed()
            d = get_stable_distance_reading() / 1000  # m
            time.sleep(0.01)
            readings.append((d, yaw))
        # เลือกค่าที่ใกล้ที่สุดในช่วง 0.1-0.6m
        valid = [item for item in readings if 0.1 < item[0] < 0.5]
        if valid:
            min_d, min_yaw = min(valid, key=lambda x: x[0])
            pillar_yaws[i] = min_yaw
            pillar_ds[i] = min_d
        else:
            pillar_yaws[i] = None
            pillar_ds[i] = None

    # คำนวณขอบเขต tile ตามเสาที่หาได้ (เหมือนเดิม)
    min_x = x_now - tile_size/2
    max_x = x_now + tile_size/2
    min_y = (y_now - tile_size/2)
    max_y = (y_now + tile_size/2)

    if way[0] == 0 and tof_wall[0] is not None:
        min_y = y_now - tof_wall[0]/1000
        max_y = min_y + tile_size
    elif pillar_ds[0] is not None:
        rad = math.radians(pillar_yaws[0])
        px = x_now + pillar_ds[0] * math.cos(rad)
        py = y_now + pillar_ds[0] * math.sin(rad)
        min_y = py
        max_y = min_y + tile_size

    if way[2] == 0 and tof_wall[2] is not None:
        max_y = y_now + tof_wall[2]/1000
        min_y = max_y - tile_size
    elif pillar_ds[2] is not None:
        rad = math.radians(pillar_yaws[2])
        px = x_now + pillar_ds[2] * math.cos(rad)
        py = y_now + pillar_ds[2] * math.sin(rad)
        max_y = py
        min_y = max_y - tile_size

    if way[1] == 0 and tof_wall[1] is not None:
        max_x = x_now + tof_wall[1]/1000
        min_x = max_x - tile_size
    elif pillar_ds[1] is not None:
        rad = math.radians(pillar_yaws[1])
        px = x_now + pillar_ds[1] * math.cos(rad)
        py = y_now + pillar_ds[1] * math.sin(rad)
        max_x = px
        min_x = max_x - tile_size

    if way[3] == 0 and tof_wall[3] is not None:
        min_x = x_now - tof_wall[3]/1000
        max_x = min_x + tile_size
    elif pillar_ds[3] is not None:
        rad = math.radians(pillar_yaws[3])
        px = x_now + pillar_ds[3] * math.cos(rad)
        py = y_now + pillar_ds[3] * math.sin(rad)
        min_x = px
        max_x = min_x + tile_size

    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2

    print(f"🟦 Pillar min/max: x={min_x:.3f}~{max_x:.3f}, y={min_y:.3f}~{max_y:.3f}")
    print(f"🎯 Pillar center: ({center_x:.3f}, {center_y:.3f})")

    return (center_x, center_y)

# def get_stable_distance_reading():
#     """
#     อ่านค่าระยะทางแบบ stable โดยรอให้ค่าคงที่
#     Returns:
#         float: ค่าระยะทางที่ stable
#     """
#     global lastest_distance  # <--- แก้ตรงนี้ให้ตรงกับตัวแปรที่ median filter update
#     stable_readings = []
#     for i in range(5):
#         stable_readings.append(lastest_distance[0])
#         time.sleep(0.02)
#     stable_value = statistics.median(stable_readings)
#     variance = max(stable_readings) - min(stable_readings)
#     if variance > 100:
#         print(f"⚠️  ToF ไม่เสถียร: variance={variance:.0f}mm, readings={stable_readings}")
#     return stable_value


def move_to_center(ep_chassis, target_x, target_y, get_position_func, tolerance=0.05, max_time=5.0):
    """
    เคลื่อนที่ไปยัง (target_x, target_y) ทีเดียวด้วย PID 2 แกน
    get_position_func: ฟังก์ชันคืนค่าตำแหน่งปัจจุบัน (x, y)
    """
    global move_pid_x, move_pid_y

    # สร้าง PID ถ้ายังไม่มี
    if move_pid_x is None:
        move_pid_x = PID(kp=2.0*0.6, ki=0.01*0.6, kd=0)
    if move_pid_y is None:
        move_pid_y = PID(kp=2.0*0.6, ki=0.01*0.6, kd=0)

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
    correct_robot_orientation(ep_chassis, target_yaw=0)
    ep_chassis.drive_speed(x=0, y=0, z=0)
    correct_robot_orientation(ep_chassis, target_yaw=0)