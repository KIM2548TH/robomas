# -*-coding:utf-8-*-
import statistics
import time
import math
from .pid import PID, TurnPID

# ตัวแปร global สำหรับ PID และ tracking
latest_chassis_position = [0, 0, 0]
latest_chassis_attitude = [0, 0, 0]  # **เพิ่ม: [yaw, pitch, roll]**
move_pid_x = None
move_pid_y = None
turn_pid = None  # **เพิ่ม: PID สำหรับการหมุน**
lastest_distance = [0] 

def sub_chassis_position(position_info):
    """Callback สำหรับตำแหน่ง chassis"""
    global latest_chassis_position
    # **แก้ไข: แปลงเป็น list เสมอ**
    latest_chassis_position = list(position_info)

def sub_chassis_attitude(attitude_info):
    """Callback สำหรับท่าทาง chassis (yaw, pitch, roll)"""
    global latest_chassis_attitude
    # **แก้ไข: แปลงเป็น list เสมอ**
    latest_chassis_attitude = list(attitude_info)

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
        move_pid_x = PID(kp=2.0, ki=0.01, kd=0.003)
    if move_pid_y is None:
        move_pid_y = PID(kp=2.0, ki=0.01, kd=0.003)
    
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
        min_y = y_now - tile_size/2
        max_y = y_now + tile_size/2

        if 'left' in wall:
            min_y = y_now - wall['left']
            max_y = min_y + tile_size
        elif 'right' in wall:
            max_y = y_now + wall['right']
            min_y = max_y - tile_size

        if 'front' in wall:
            max_x = x_now + wall['front']
            min_x = max_x - tile_size
        elif 'back' in wall:
            min_x = x_now - wall['back']
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

            move_direction_pid(ep_chassis, 'x+', center_x - x_now)
            correct_robot_orientation(ep_chassis, target_yaw=0)
            move_direction_pid(ep_chassis, 'y+', center_y - y_now)
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

    # ตรวจสอบค่าก่อน
    if latest_chassis_position is None or len(latest_chassis_position) < 2:
        print(latest_chassis_position)
        raise ValueError("latest_chassis_position ยังไม่มีข้อมูล")

    # กำหนดตรงนี้แค่ครั้งเดียว!
    x_now, y_now = latest_chassis_position[0], latest_chassis_position[1]

    # [ซ้ายบน, ขวาบน, ซ้ายล่าง, ขวาล่าง]
    pillar_yaws = [None, None, None, None]
    pillar_ds = [None, None, None, None]

    sweep_map = [
        sweep_angles_list(-135, -135, 1),  # ซ้ายบน
        sweep_angles_list(135, 135, 1),    # ขวาบน
        sweep_angles_list(-45, -45, 1),    # ซ้ายล่าง
        sweep_angles_list(45, 45, 1),      # ขวาล่าง
    ]

    skip = [False, False, False, False]
    if way[0] == 0:  # มีกำแพงซ้าย
        skip[0] = True  # ข้ามซ้ายบน
        skip[2] = True  # ข้ามซ้ายล่าง
    if way[2] == 0:  # มีกำแพงขวา
        skip[1] = True  # ข้ามขวาบน
        skip[3] = True  # ข้ามขวาล่าง
    if way[1] == 0:  # มีกำแพงหน้า
        skip[0] = True  # ข้ามซ้ายบน
        skip[1] = True  # ข้ามขวาบน
    if way[3] == 0:  # มีกำแพงหลัง
        skip[2] = True  # ข้ามซ้ายล่าง
        skip[3] = True  # ข้ามขวาล่าง

    for i in range(4):
        if skip[i]:
            print(f"🚧 ข้าม sweep {['ซ้ายบน','ขวาบน','ซ้ายล่าง','ขวาล่าง'][i]} เพราะมี/ติดกำแพง")
            pillar_yaws[i] = None
            pillar_ds[i] = None
            continue
        min_d = None
        min_yaw = None
        for yaw in sweep_map[i]:
            ep_gimbal.moveto(pitch=-6, yaw=yaw, pitch_speed=350, yaw_speed=350).wait_for_completed()
            d = get_stable_distance_reading() / 1000  # m
            time.sleep(0.01)
            if 0.05 < d < 0.6:
                if (min_d is None) or (d < min_d):
                    min_d = d
                    min_yaw = yaw
        pillar_yaws[i] = min_yaw
        pillar_ds[i] = min_d

    min_x = x_now - tile_size/2
    max_x = x_now + tile_size/2
    min_y = y_now - tile_size/2
    max_y = y_now + tile_size/2

    # ใช้ข้อมูลกำแพงถ้ามี
    if way[0] == 0 and tof_wall[0] is not None:
        min_y = y_now - tof_wall[0]
        max_y = min_y + tile_size
    elif pillar_ds[0] is not None:
        rad = math.radians(pillar_yaws[0])
        px = x_now + pillar_ds[0] * math.cos(rad)
        py = y_now + pillar_ds[0] * math.sin(rad)
        min_y = py
        max_y = min_y + tile_size

    if way[2] == 0 and tof_wall[2] is not None:
        max_y = y_now + tof_wall[2]
        min_y = max_y - tile_size
    elif pillar_ds[2] is not None:
        rad = math.radians(pillar_yaws[2])
        px = x_now + pillar_ds[2] * math.cos(rad)
        py = y_now + pillar_ds[2] * math.sin(rad)
        max_y = py
        min_y = max_y - tile_size

    if way[1] == 0 and tof_wall[1] is not None:
        max_x = x_now + tof_wall[1]
        min_x = max_x - tile_size
    elif pillar_ds[1] is not None:
        rad = math.radians(pillar_yaws[1])
        px = x_now + pillar_ds[1] * math.cos(rad)
        py = y_now + pillar_ds[1] * math.sin(rad)
        max_x = px
        min_x = max_x - tile_size

    if way[3] == 0 and tof_wall[3] is not None:
        min_x = x_now - tof_wall[3]
        max_x = min_x + tile_size
    elif pillar_ds[3] is not None:
        rad = math.radians(pillar_yaws[3])
        px = x_now + pillar_ds[3] * math.cos(rad)
        py = y_now + pillar_ds[3] * math.sin(rad)
        min_x = px
        max_x = min_x + tile_size

    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2

    return (center_x, center_y)

def get_stable_distance_reading():
    """
    อ่านค่าระยะทางแบบ stable โดยรอให้ค่าคงที่
    Returns:
        float: ค่าระยะทางที่ stable
    """
    global lastest_distance  # <--- แก้ตรงนี้ให้ตรงกับตัวแปรที่ median filter update
    stable_readings = []
    for i in range(5):
        stable_readings.append(lastest_distance[0])
        time.sleep(0.02)
    stable_value = statistics.median(stable_readings)
    variance = max(stable_readings) - min(stable_readings)
    if variance > 100:
        print(f"⚠️  ToF ไม่เสถียร: variance={variance:.0f}mm, readings={stable_readings}")
    return stable_value