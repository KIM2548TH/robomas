# -*-coding:utf-8-*-
import robomaster
from robomaster import robot
import time
import math
import csv
from collections import deque
import os
from .pid import PID, TurnPID

# ตัวแปรสำหรับเก็บข้อมูล subscription
current_pos = [0, 0, 0]
current_attitude = [0, 0, 0]  # [pitch, roll, yaw]
position_history = deque(maxlen=100)
attitude_history = deque(maxlen=50)
movement_stats = {
    'distance_moved': 0,
    'speed': 0,
    'stable_count': 0,
    'last_update': time.time()
}

# **เพิ่ม: ระบบตรวจจับทางตัน**
wall_detection_stats = {
    'last_position': None,
    'movement_attempts': 0,
    'blocked_directions': set(),
    'detection_threshold': 0.05,  # หากเคลื่อนที่น้อยกว่า 5cm ใน 1 วินาที = ติดกำแพง
    'time_threshold': 1.0
}

# ตัวแปรระบบ
ep_robot = None
ep_chassis = None
move_pid = None
turn_pid_controller = None
move_writer = None
turn_writer = None
move_file = None
turn_file = None
move_distance = 0.5

# **เพิ่ม: ระบบ calibration ทิศทาง**
orientation_calibration = {
    'is_calibrated': False,
    'initial_yaw': 0,  # มุม yaw เริ่มต้น
    'initial_direction': 'y+',  # ทิศทางจริงที่หุ่นยนต์หันหน้าไปตอนเริ่มต้น
    'direction_offset': 0  # offset ระหว่าง yaw กับทิศทางจริง
}

# ===== SUBSCRIPTION HANDLERS =====

def sub_position_handler(position_info):
    """Handler สำหรับ subscription ตำแหน่ง"""
    global current_pos, position_history, movement_stats
    
    x, y, z = position_info
    prev_pos = current_pos.copy()
    current_pos[:] = [x, y, z]
    
    if len(position_history) > 0:
        last_pos = position_history[-1]
        distance_delta = math.sqrt(
            (x - last_pos['x'])**2 + 
            (y - last_pos['y'])**2
        )
        movement_stats['distance_moved'] += distance_delta
        
        time_delta = time.time() - last_pos['time']
        if time_delta > 0:
            movement_stats['speed'] = distance_delta / time_delta
        
        if distance_delta < 0.001:
            movement_stats['stable_count'] += 1
        else:
            movement_stats['stable_count'] = 0
    
    position_history.append({
        'time': time.time(),
        'x': x, 'y': y, 'z': z
    })
    
    movement_stats['last_update'] = time.time()

def sub_attitude_handler(attitude_info):
    """Handler สำหรับ subscription ท่าทาง"""
    global current_attitude, attitude_history
    
    yaw, pitch, roll = attitude_info
    current_attitude[:] = [pitch, roll, yaw]
    
    attitude_history.append({
        'time': time.time(),
        'yaw': yaw, 'pitch': pitch, 'roll': roll
    })





# ===== CORE MOVEMENT FUNCTIONS =====

def move_forward(distance=None):
    """เดินไปข้างหน้าระยะทางที่กำหนด"""
    global move_distance, movement_stats, position_history
    
    target_distance = distance if distance is not None else move_distance
    
    print(f"🚶 เดินไปข้างหน้า {target_distance:.2f}m")
    
    # รีเซ็ต PID และสถิติ
    move_pid.reset()
    movement_stats = {
        'distance_moved': 0, 'speed': 0, 'stable_count': 0,
        'last_update': time.time()
    }
    position_history.clear()
    
    start_pos = current_pos.copy()
    tolerance = 0.01
    stable_iterations = 0
    last_progress_time = time.time()
    
    print(f"📍 เริ่มต้นที่: ({start_pos[0]:.3f}, {start_pos[1]:.3f})")
    
    while True:
        dx = current_pos[0] - start_pos[0]
        dy = current_pos[1] - start_pos[1]
        current_distance = math.sqrt(dx*dx + dy*dy)
        
        # แสดงความคืบหน้า
        current_time = time.time()
        if current_time - last_progress_time >= 0.5:
            progress = min(100, (current_distance / target_distance) * 100)
            # print(f"   📊 {current_distance:.3f}m/{target_distance:.2f}m ({progress:.1f}%) "
                #   f"เร็ว:{movement_stats['speed']:.3f}m/s")
            last_progress_time = current_time
        
        error_current = target_distance - current_distance
        
        if abs(error_current) < tolerance:
            stable_iterations += 1
            if stable_iterations >= 30:
                break
        else:
            stable_iterations = 0
        
        if movement_stats['stable_count'] > 100:
            print("⚠️ ตรวจพบการติดขัด - หยุดการเคลื่อนที่")
            break
        
        pid_output, p, i, d, e, t = move_pid.compute(target_distance, current_distance)
        speed = pid_output
        
        # บันทึกข้อมูล
        if move_writer:
            move_writer.writerow([
                current_pos[0], current_pos[1], p, i, d, pid_output, e, t,
                movement_stats['speed'], movement_stats['stable_count'], current_distance
            ])
        
        ep_chassis.drive_speed(x=speed, y=0, z=0)
        time.sleep(0.02)
    
    ep_chassis.drive_speed(x=0, y=0, z=0)
    final_distance = math.sqrt(
        (current_pos[0] - start_pos[0])**2 + 
        (current_pos[1] - start_pos[1])**2
    )
    print(f"✅ เดินเสร็จ {final_distance:.3f}m")
    time.sleep(0.2)
    return final_distance

def turn_right(angle=90):
    """หมุนขวาตามมุมที่กำหนด"""
    return _turn("right", angle)

def turn_left(angle=90):
    """หมุนซ้ายตามมุมที่กำหนด"""
    return _turn("left", angle)

def _turn(direction, angle):
    """ฟังก์ชันหมุนหลัก"""
    global current_attitude, attitude_history
    
    if direction.lower() == "right":
        print(f"↪️ หมุนขวา {angle}°")
        angle_multiplier = 1
    elif direction.lower() == "left":
        print(f"↩️ หมุนซ้าย {angle}°")
        angle_multiplier = -1
    else:
        print("❌ ทิศทางไม่ถูกต้อง!")
        return
    
    # รีเซ็ต PID
    turn_pid_controller.reset()
    attitude_history.clear()
    
    time.sleep(0.3)
    start_yaw = current_attitude[2]
    target_yaw = start_yaw + (angle * angle_multiplier)
    
    # ปรับค่า yaw ให้อยู่ในช่วง -180 ถึง 180
    if target_yaw < -180: 
        target_yaw += 360
    elif target_yaw > 180: 
        target_yaw -= 360
    
    print(f"🎯 มุมเริ่มต้น: {start_yaw:.2f}°, เป้าหมาย: {target_yaw:.2f}°")
    
    tolerance = 0.5
    stable_count = 0
    last_display_time = time.time()
    
    while True:
        current_yaw = current_attitude[2]
        
        pid_output, angle_error, p, i, d = turn_pid_controller.compute(target_yaw, current_yaw)
        turn_speed = pid_output
        
        # แสดงข้อมูล
        current_time = time.time()
        if current_time - last_display_time >= 0.2:
            # print(f"   🔄 ปัจจุบัน: {current_yaw:.1f}° เป้าหมาย: {target_yaw:.1f}° "
                #   f"ผิดพลาด: {angle_error:.1f}°")
            last_display_time = current_time
        
        # บันทึกข้อมูล
        if turn_writer:
            turn_writer.writerow([time.time(), current_yaw, target_yaw, angle_error, p, i, d, pid_output])
        
        if abs(angle_error) < tolerance:
            stable_count += 1
            if stable_count >= 15:
                break
        else:
            stable_count = 0
        
        ep_chassis.drive_speed(x=0, y=0, z=turn_speed)
        time.sleep(0.02)
    
    ep_chassis.drive_speed(x=0, y=0, z=0)
    time.sleep(0.5)
    
    # สรุปผล
    end_yaw = current_attitude[2]
    actual_change = end_yaw - start_yaw
    
    if actual_change > 180: 
        actual_change -= 360
    elif actual_change < -180: 
        actual_change += 360
    
    expected_change = angle * angle_multiplier
    error_from_target = abs(actual_change - expected_change)
    
    print(f"🎯 มุมหลังหมุน: {end_yaw:.2f}°")
    print(f"📏 การเปลี่ยนแปลงจริง: {actual_change:.2f}°")
    print(f"📏 ความคลาดเคลื่อน: {error_from_target:.2f}°")
    print("✅ หมุนเสร็จสิ้น")

# ===== UTILITY FUNCTIONS =====

def get_current_position():
    """ดึงตำแหน่งปัจจุบัน"""
    return current_pos[0], current_pos[1]

def get_current_orientation():
    """ดึงมุมปัจจุบัน (yaw)"""
    return current_attitude[2]

def get_movement_stats():
    """ดึงสถิติการเคลื่อนที่"""
    return movement_stats.copy()

def set_move_distance(distance):
    """กำหนดระยะทางการเคลื่อนที่เริ่มต้น"""
    global move_distance
    move_distance = distance
    print(f"📏 กำหนดระยะทางการเคลื่อนที่เริ่มต้น: {distance}m")

# ===== INITIALIZATION =====

def calibrate_initial_orientation(facing_direction='x+'):
    """
    Calibrate ทิศทางเริ่มต้นของหุ่นยนต์
    
    Args:
        facing_direction (str): ทิศทางจริงที่หุ่นยนต์หันหน้าไป ('x+', 'x-', 'y+', 'y-')
    """
    global orientation_calibration
    
    current_yaw = current_attitude[2]
    
    # แมปทิศทางจริงกับมุม - เปลี่ยนให้ตรงกับ main.py
    direction_to_angle = {
        'x+': 0,    # ตะวันออก (เริ่มต้น)
        'y-': 90,   # ใต้
        'x-': 180,  # ตะวันตก
        'y+': -90   # เหนือ
    }
    
    expected_angle = direction_to_angle.get(facing_direction, 0)
    offset = expected_angle - current_yaw
    
    # ปรับ offset ให้อยู่ในช่วง -180 ถึง 180
    while offset > 180:
        offset -= 360
    while offset < -180:
        offset += 360
    
    orientation_calibration = {
        'is_calibrated': True,
        'initial_yaw': current_yaw,
        'initial_direction': facing_direction,
        'direction_offset': offset
    }
    
    print(f"🧭 Calibrate ทิศทาง:")
    print(f"   - Yaw ปัจจุบัน: {current_yaw:.1f}°")
    print(f"   - ทิศทางจริง: {facing_direction}")
    print(f"   - มุมที่คาดหวัง: {expected_angle}°")
    print(f"   - Offset: {offset:.1f}°")
    print("✅ Calibration เสร็จสิ้น")





def init_movement_system():
    """เริ่มต้นระบบการเคลื่อนที่"""
    global ep_robot, ep_chassis, move_pid, turn_pid_controller
    global current_pos, current_attitude, move_writer, turn_writer
    global move_file, turn_file, move_distance
    
    print("🤖 เริ่มระบบการเคลื่อนที่...")
    
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis

    print("ตั้งค่า LED...")
    ep_robot.led.set_led(comp="all", r=255, g=255, b=255, effect="scrolling")
    time.sleep(1) 

    # เริ่มต้น PID
    move_pid = PID(kp=2, ki=0.01, kd=0.03) 
    turn_pid_controller = TurnPID(kp=5, ki=0, kd=0.5)

    move_distance = 0.5
    current_pos = [0, 0, 0]
    current_attitude = [0, 0, 0]

    # ตั้งค่า subscription
    print("📡 ตั้งค่า subscription...")
    ep_chassis.sub_position(freq=50, callback=sub_position_handler)
    ep_chassis.sub_attitude(freq=20, callback=sub_attitude_handler)
    
    time.sleep(2)
    
    print(f"📍 ตำแหน่งเริ่มต้น: ({current_pos[0]:.3f}, {current_pos[1]:.3f})")
    print(f"🧭 ท่าทางเริ่มต้น: yaw {current_attitude[2]:.1f}°")
    
    # **เพิ่ม: Calibrate ทิศทางเริ่มต้น**
    print("\n🧭 กำลัง calibrate ทิศทางเริ่มต้น...")
    calibrate_initial_orientation('x+')  # เริ่มต้นด้วย x+ (ตะวันออก) ตรงกับ main.py
    
    # สร้างไฟล์ log
    output_dir = "csv_pid"
    os.makedirs(output_dir, exist_ok=True)
    
    move_log_filename = os.path.join(output_dir, "movement_log.csv")
    turn_log_filename = os.path.join(output_dir, "turn_log.csv")
    
    move_file = open(move_log_filename, mode='w', newline='')
    turn_file = open(turn_log_filename, mode='w', newline='')
    
    move_writer = csv.writer(move_file)
    turn_writer = csv.writer(turn_file)
    
    move_writer.writerow([
        "x", "y", "p", "i", "d", "pid_output", "error", "time",
        "speed", "stability", "distance"
    ])
    turn_writer.writerow([
        "timestamp", "current_yaw", "target_yaw", "angle_error", 
        "p_term", "i_term", "d_term", "pid_output"
    ])
    
    print("✅ ระบบการเคลื่อนที่พร้อมใช้งาน!")

def cleanup_movement_system():
    """ปิดระบบการเคลื่อนที่"""
    global ep_robot, ep_chassis, move_file, turn_file
    
    print("🏁 ปิดระบบการเคลื่อนที่...")
    
    if ep_robot:
        ep_robot.led.set_led(comp="all", effect="off")
    if ep_chassis:
        ep_chassis.drive_speed(x=0, y=0, z=0)
        ep_chassis.unsub_position()
        ep_chassis.unsub_attitude()
    
    if move_file:
        move_file.close()
    if turn_file:
        turn_file.close()
        
    if ep_robot:
        ep_robot.close()
    
    print("✅ ปิดเรียบร้อย")