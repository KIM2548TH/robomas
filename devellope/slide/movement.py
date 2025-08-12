# -*-coding:utf-8-*-

import time
import math
from .pid import PID, TurnPID

# ตัวแปร global สำหรับ PID และ tracking
latest_chassis_position = [0, 0, 0]
latest_chassis_attitude = [0, 0, 0]  # **เพิ่ม: [yaw, pitch, roll]**
move_pid_x = None
move_pid_y = None
turn_pid = None  # **เพิ่ม: PID สำหรับการหมุน**

def sub_chassis_position(position_info):
    """Callback สำหรับตำแหน่ง chassis"""
    global latest_chassis_position
    if isinstance(position_info, (tuple, list)):
        latest_chassis_position = list(position_info)
    else:
        latest_chassis_position = [position_info[0], position_info[1], position_info[2]]

def sub_chassis_attitude(attitude_info):
    """Callback สำหรับท่าทาง chassis (yaw, pitch, roll)"""
    global latest_chassis_attitude
    if isinstance(attitude_info, (tuple, list)):
        latest_chassis_attitude = list(attitude_info)
    else:
        latest_chassis_attitude = [attitude_info[0], attitude_info[1], attitude_info[2]]

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
        turn_pid = TurnPID(kp=2.0, ki=0.01, kd=0.03)
    
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
    if abs(yaw_error) < 2.0:
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
            if stable_count >= 10:
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

# ตัวอย่างการใช้งานฟังก์ชันใหม่
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
    
    # **ตรวจสอบและแปลง latest_chassis_position เป็น list**
    if not isinstance(latest_chassis_position, list):
        latest_chassis_position = list(latest_chassis_position)
    
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
        move_pid_x = PID(kp=2.0, ki=0.01, kd=0.003)
    if move_pid_y is None:
        move_pid_y = PID(kp=2.0, ki=0.01, kd=0.003)
    
    # รีเซ็ต PID
    move_pid_x.reset()
    move_pid_y.reset()
    
    # **ใช้ list() แทน .copy()**
    start_pos = list(latest_chassis_position)
    
    # คำนวณตำแหน่งเป้าหมาย
    target_x = start_pos[0] + (distance * dir_info['x'])
    target_y = start_pos[1] + (distance * dir_info['y'])
    
    tolerance = 0.01  # 1cm
    stable_iterations = 0
    max_iterations = 200
    iteration = 0
    
    print(f"📍 เริ่มต้นที่: ({start_pos[0]:.3f}, {start_pos[1]:.3f})")
    print(f"🎯 เป้าหมาย: ({target_x:.3f}, {target_y:.3f})")
    
    while iteration < max_iterations:
        iteration += 1
        
        # **ตรวจสอบ latest_chassis_position อีกครั้ง**
        if not isinstance(latest_chassis_position, list):
            latest_chassis_position = list(latest_chassis_position)
        
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
            print(f"   📊 {current_distance:.3f}m/{distance:.3f}m ({progress:.1f}%) error:{total_error:.3f}m")
        
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

def move_direction_pid_wall(ep_chassis, direction, distance):
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
    
    # **ตรวจสอบและแปลง latest_chassis_position เป็น list**
    if not isinstance(latest_chassis_position, list):
        latest_chassis_position = list(latest_chassis_position)
    
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
        move_pid_x = PID(kp=2.0, ki=0.01, kd=0.003)
    if move_pid_y is None:
        move_pid_y = PID(kp=2.0, ki=0.01, kd=0.003)
    
    # รีเซ็ต PID
    move_pid_x.reset()
    move_pid_y.reset()
    
    # **ใช้ list() แทน .copy()**
    start_pos = list(latest_chassis_position)
    
    # คำนวณตำแหน่งเป้าหมาย
    target_x = start_pos[0] + (distance * dir_info['x'])
    target_y = start_pos[1] + (distance * dir_info['y'])
    
    tolerance = 0.05  # 1cm
    stable_iterations = 0
    max_iterations = 200
    iteration = 0
    
    print(f"📍 เริ่มต้นที่: ({start_pos[0]:.3f}, {start_pos[1]:.3f})")
    print(f"🎯 เป้าหมาย: ({target_x:.3f}, {target_y:.3f})")
    
    while iteration < max_iterations:
        iteration += 1
        
        # **ตรวจสอบ latest_chassis_position อีกครั้ง**
        if not isinstance(latest_chassis_position, list):
            latest_chassis_position = list(latest_chassis_position)
        
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
            print(f"   📊 {current_distance:.3f}m/{distance:.3f}m ({progress:.1f}%) error:{total_error:.3f}m")
        
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