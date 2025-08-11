import time
import sys
import os
import math

# เพิ่ม path สำหรับ import module
sys.path.append(os.path.join(os.path.dirname(__file__), 'controler'))

from controler.movement import (
    init_movement_system, 
    cleanup_movement_system,
    move_forward, 
    turn_right, 
    turn_left,
    get_current_position,
    get_current_orientation, 
    get_movement_stats,
    set_move_distance,
    # **import ระบบ calibration**
    calibrate_initial_orientation,
    # **ลบ convert_blocked_directions_to_compass**
)
from controler.map import Graph, a_star_search, Node, calculate_direction
from controler.check_wall import (
    move_gimbal,
    initialize_sensors,
    cleanup_sensors,
    detect_walls_with_gimbal,
    convert_gimbal_result_to_blocked_directions
)

class MazeRunner:
    def __init__(self):
        self.maze_map = Graph()
        self.current_node_id = None
        self.path_history = []
        self.move_distance = 0.5
        set_move_distance(self.move_distance)
        
        # **เพิ่ม: ระบบติดตามทิศทาง**
        self.current_facing_direction = 'x+'  # เริ่มต้นหันหน้าทิศตะวันออก
        self.direction_history = []  # บันทึกประวัติการหมุน

        print(f"🧭 เริ่มต้นหันหน้าไปทิศ: {self.current_facing_direction} (ตะวันออก)")

    def update_facing_direction(self, turn_direction, angle=90):
        """
        อัปเดตทิศทางหลังจากหมุน - รองรับทุกมุม
        
        Args:
            turn_direction (str): 'right' หรือ 'left'
            angle (float): มุมที่หมุน (องศา) - รองรับ 90, 180, 270, 360
        """
        # ลำดับทิศทางสำหรับการหมุนขวา - เริ่มต้นด้วย x+ (ตะวันออก)
        right_sequence = ['x+', 'y+', 'x-', 'y-']  # ตะวันออก → ใต้ → ตะวันตก → เหนือ
        
        current_index = right_sequence.index(self.current_facing_direction)
        
        # แยกกรณีทุกมุมแยกชัด
        if turn_direction.lower() == 'right':
            if angle == 90:
                new_index = (current_index + 1) % 4
            elif angle == 180:
                new_index = (current_index + 2) % 4
            elif angle == 270:
                new_index = (current_index + 3) % 4
            elif angle == 360:
                new_index = current_index  # กลับทิศเดิม
            else:
                print(f"❌ มุมการหมุนขวาไม่ถูกต้อง: {angle}°")
                return
        elif turn_direction.lower() == 'left':
            if angle == 90:
                new_index = (current_index - 1) % 4
            elif angle == 180:
                new_index = (current_index - 2) % 4
            elif angle == 270:
                new_index = (current_index - 3) % 4
            elif angle == 360:
                new_index = current_index  # กลับทิศเดิม
            else:
                print(f"❌ มุมการหมุนซ้ายไม่ถูกต้อง: {angle}°")
                return
        else:
            print(f"❌ ทิศทางการหมุนไม่ถูกต้อง: {turn_direction}")
            return
        
        old_direction = self.current_facing_direction
        self.current_facing_direction = right_sequence[new_index]
        
        # บันทึกประวัติ
        self.direction_history.append({
            'action': f'turn_{turn_direction}',
            'angle': angle,
            'from': old_direction,
            'to': self.current_facing_direction,
            'node': self.current_node_id
        })
        
        direction_names = {
            'y+': 'เหนือ', 'y-': 'ใต้', 'x+': 'ตะวันออก', 'x-': 'ตะวันตก'
        }
        
        print(f"✅ หมุน{turn_direction} {angle}°: {direction_names[old_direction]} → {direction_names[self.current_facing_direction]}")

    def verify_direction_from_movement(self, from_node_id, to_node_id):
        """
        ตรวจสอบทิศทางจากการเคลื่อนที่ระหว่างโหนด
        """
        if from_node_id is None or to_node_id is None:
            return
        
        from_node = self.maze_map.nodes[from_node_id]
        to_node = self.maze_map.nodes[to_node_id]
        
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        
        # คำนวณทิศทางจากการเคลื่อนที่
        if abs(dx) > abs(dy):
            actual_direction = 'x+' if dx > 0 else 'x-'
        else:
            actual_direction = 'y+' if dy > 0 else 'y-'
        
        direction_names = {
            'y+': 'เหนือ', 'y-': 'ใต้', 'x+': 'ตะวันออก', 'x-': 'ตะวันตก'
        }
        
        print(f"📊 ตรวจสอบทิศทาง:")
        print(f"   - การเปลี่ยนแปลง: dx={dx:.3f}, dy={dy:.3f}")
        print(f"   - ทิศทางจากการเคลื่อนที่: {actual_direction} ({direction_names[actual_direction]})")
        print(f"   - ทิศทางที่จำไว้: {self.current_facing_direction} ({direction_names[self.current_facing_direction]})")
        
        if actual_direction == self.current_facing_direction:
            print("✅ ทิศทางถูกต้อง!")
        else:
            print("⚠️ ทิศทางไม่ตรงกัน! แก้ไข...")
            self.current_facing_direction = actual_direction
            print(f"🔄 ปรับทิศทางเป็น: {actual_direction} ({direction_names[actual_direction]})")
        
        # บันทึกการตรวจสอบ
        self.direction_history.append({
            'action': 'verify_movement',
            'from_node': from_node_id,
            'to_node': to_node_id,
            'expected': self.current_facing_direction,
            'actual': actual_direction,
            'correct': actual_direction == self.current_facing_direction
        })
    
    def get_real_position(self):
        """ดึงตำแหน่งจริงจากเซ็นเซอร์"""
        return get_current_position()
        
    def add_node_here(self, is_exit=False, auto_detect_walls=True):
        """
        เพิ่มโหนดที่ตำแหน่งปัจจุบัน พร้อมตรวจจับทางตันด้วย gimbal
        """
        x, y = self.get_real_position()
        print(f"📍 ตำแหน่งจากเซ็นเซอร์: x={x:.3f}, y={y:.3f}")
        
        # ตรวจจับทางตันด้วย gimbal
        detected_walls = []
        if auto_detect_walls:
            print("🔍 เริ่มตรวจจับทางตันด้วย gimbal...")
            # ใช้ฟังก์ชัน detect_walls_with_gimbal
            from controler.movement import ep_robot
            detected_walls = detect_walls_with_gimbal(ep_robot, self.current_facing_direction)
            
        # รวมทางตันที่ตรวจพบกับที่ระบุ manual
        all_blocked = []
        if detected_walls[1]:
            all_blocked.extend(detected_walls[1])  
            print(f"🤖 ตรวจพบทางตันด้วย gimbal: {detected_walls[1]}")

        if all_blocked:
            print(f"🚫 ทางตันรวม: {all_blocked}")
        
        # แสดงสถิติการเคลื่อนที่
        stats = get_movement_stats()
        print(f"🚀 ความเร็ว: {stats['speed']:.3f}m/s")
        
        display_x = round(x, 1)
        display_y = round(y, 1)
        
        node_id = self.maze_map.add_node(x=x, y=y, is_exit=is_exit, 
                                       blocked_directions=all_blocked)
        print(f"📍 เพิ่มโหนด {node_id} ที่ ({display_x}, {display_y})")
        print(detected_walls)
        return node_id,detected_walls[0]

    def find_existing_node(self):
        """หาโหนดเดิมที่ใกล้ตำแหน่งปัจจุบัน"""
        x, y = self.get_real_position()
        
        print(f"🔍 ตรวจสอบตำแหน่ง: x={x:.3f}, y={y:.3f}")
        
        best_node = None
        min_diff = float('inf')
        tolerance = 0.3  # 30cm
        
        for node_id, node in self.maze_map.nodes.items():
            x_diff = abs(node.x - x)
            y_diff = abs(node.y - y)
            total_diff = x_diff + y_diff
            
            print(f"   โหนด {node_id} ({node.x:.3f}, {node.y:.3f}): "
                  f"X:{x_diff:.3f} Y:{y_diff:.3f}", end="")
            
            if x_diff <= tolerance and y_diff <= tolerance and total_diff < min_diff:
                min_diff = total_diff
                best_node = node_id
                print(f" ← เจอ!")
            else:
                print("")
        
        return best_node
    

    
    def explore_maze(self):
        """สำรวจเขาวงกตแบบอัตโนมัติ - เช็คทางก่อนแล้วไปต่อ"""
        print("🏁 เริ่มสำรวจเขาวงกต!")
        
        # จุดเริ่มต้น - สแกนกำแพงเลย
        start_node,wall = self.add_node_here(auto_detect_walls=True)
        self.path_history.append(start_node)
        self.current_node_id = start_node
        max_steps = 55
        print(wall,656565656565)

        current_node = self.maze_map.nodes[self.current_node_id]
        # current_node.blocked_directions.extend('x-')  # เพิ่มทางตัน x- (ตะวันตก) ให้โหนดเริ่มต้น
        
        for step in range(1, max_steps + 1):
            print(f"\n--- ขั้นตอนที่ {step} ---")
            
            # ✅ 1. เช็คโหนดปัจจุบัน
            current_node = self.maze_map.nodes[self.current_node_id]
            print(f"📍 ตรวจสอบโหนดปัจจุบัน {self.current_node_id}:")
            print(f"   - ทางตัน: {wall}")
            
            # ✅ 2. เลือกทิศทางจากข้อมูล wall
            chosen_direction = ''
            if wall:
                if wall[0] == 1:
                    chosen_direction = 'left'
                elif wall[1] == 1:
                    chosen_direction = 'front'
                elif wall[2] == 1:
                    chosen_direction = 'right'
                elif wall[0] + wall[1] + wall[2] == 0:
                    chosen_direction = "back"
            
            # ✅ 3. ถ้าไม่มีทางไป หรือ back ให้ใช้ A* หาทางใหม่
            if chosen_direction == "back" or chosen_direction == '':
                print("🚫 ทางตันหรือไม่มีทาง - ใช้ A* หาเป้าหมายใหม่")
                
                # หาโหนดเป้าหมาย (neighbors + blocked_directions < 4)
                target_node_id = self.find_exploration_target()
                
                if target_node_id is None:
                    print("🎉 สำรวจครบแล้ว!")
                    break
                
                # ใช้ A* หาเส้นทาง
                print(f"🎯 เป้าหมาย: โหนด {target_node_id}")
                path, distance = a_star_search(self.maze_map, self.current_node_id, target_node_id)
                
                if path is None:
                    print("❌ ไม่เจอทาง!")
                    break
                
                print(f"🗺️ เส้นทาง A*: {' -> '.join(map(str, path))}")
                
                # เดินตามเส้นทาง A*
                self.navigate_to_target(path)
                
                # อัปเดตตำแหน่งปัจจุบัน
                self.current_node_id = target_node_id
                
                # สแกน wall ใหม่ที่เป้าหมาย
                print("🔍 สแกนใหม่ที่เป้าหมาย...")
                from controler.movement import ep_robot
                wall_data = detect_walls_with_gimbal(ep_robot, self.current_facing_direction)
                wall = wall_data[0]
                
                continue
            
            # ✅ 4. เดินตามทิศทางปกติ
            print(f"🎯 ต้องหันไป: {chosen_direction}")
            previous_node_id = self.current_node_id

            if chosen_direction == 'left':
                print("➡️ หันหน้าไปทิศซ้าย")
                turn_left(90)
                self.update_facing_direction('left', 90)
                time.sleep(0.1)
                move_forward(0.6)
            elif chosen_direction == 'right':
                print("➡️ หันหน้าไปทิศขวา")
                turn_right(90)
                self.update_facing_direction('right', 90)
                time.sleep(0.1)
                move_forward(0.6)
            elif chosen_direction == 'front':
                print("➡️ หันหน้าไปทิศหน้า")
                move_forward(0.6)

            # ✅ 5. ตรวจสอบโหนดใหม่/เดิม
            existing_node_id = self.find_existing_node()

            if existing_node_id is not None:
                print(f"🔄 เจอโหนดเดิม: {existing_node_id}")
                self.maze_map.add_edge(previous_node_id, existing_node_id, 0.6)
                self.current_node_id = existing_node_id
                
                # **ลบการสแกน wall ออก - ใช้ A* หาเป้าหมายใหม่เลย**
                target_node_id = self.find_exploration_target()
                if target_node_id is None:
                    print("🎉 สำรวจครบแล้ว!")
                    break
                    
                print(f"🎯 เป้าหมายใหม่: โหนด {target_node_id}")
                path, distance = a_star_search(self.maze_map, self.current_node_id, target_node_id)
                
                if path:
                    self.navigate_to_target(path)
                    self.current_node_id = target_node_id
                    
                    # **สแกน wall เฉพาะที่เป้าหมายใหม่เท่านั้น**
                    from controler.movement import ep_robot
                    wall_data = detect_walls_with_gimbal(ep_robot, self.current_facing_direction)
                    wall = wall_data[0]
                else:
                    # **ถ้าไม่เจอทาง A* ให้หยุด**
                    print("❌ ไม่เจอทาง A*!")
                    break
            else:
                print("🆕 ตำแหน่งใหม่")
                new_node_id, wall = self.add_node_here(auto_detect_walls=True)
                self.maze_map.add_edge(previous_node_id, new_node_id, 0.6)
                self.current_node_id = new_node_id

            self.path_history.append(self.current_node_id)
            print(f"📊 สถิติ: โหนด {len(self.maze_map.nodes)}")
            time.sleep(0.5)

        # หาทางออก
        start_node_obj = self.maze_map.nodes[start_node]
        max_distance = 0
        end_node = start_node
        
        for node_id, node in self.maze_map.nodes.items():
            if node_id != start_node:
                distance = math.sqrt((node.x - start_node_obj.x)**2 + (node.y - start_node_obj.y)**2)
                if distance > max_distance:
                    max_distance = distance
                    end_node = node_id

        self.maze_map.nodes[end_node].is_exit = True
        print(f"🎯 กำหนดโหนด {end_node} เป็นทางออก")
        
        self.show_map()
        return start_node, end_node

    def find_exploration_target(self):
        """หาโหนดเป้าหมายสำหรับสำรวจต่อ (neighbors + blocked_directions < 4)"""
        candidates = []
        
        for node_id, node in self.maze_map.nodes.items():
            if node_id == self.current_node_id:
                continue
                
            neighbors_count = len(node.connections)
            blocked_count = len(node.blocked_directions)
            total = neighbors_count + blocked_count
            
            if total < 4:
                # คำนวณระยะทางจากโหนดปัจจุบัน
                current_node = self.maze_map.nodes[self.current_node_id]
                distance = math.sqrt((node.x - current_node.x)**2 + (node.y - current_node.y)**2)
                
                candidates.append((node_id, total, distance))
                print(f"🎯 โหนด {node_id}: neighbors={neighbors_count}, blocked={blocked_count}, total={total}, distance={distance:.2f}")
        
        if not candidates:
            return None
        
        # เลือกโหนดที่ total น้อยที่สุด แล้วใกล้ที่สุด
        candidates.sort(key=lambda x: (x[1], x[2]))
        target_id = candidates[0][0]
        
        print(f"✅ เลือกเป้าหมาย: โหนด {target_id}")
        return target_id

    def navigate_to_target(self, path):
        """เดินไปยังเป้าหมาย - รวมระยะทางที่เป็นทิศเดียวกัน"""
        print(f"🚶 เดินตามเส้นทาง: {' -> '.join(map(str, path))}")
        
        i = 0
        while i < len(path) - 1:
            current_id = path[i]
            current_node = self.maze_map.nodes[current_id]
            
            # หาทิศทางแรก
            next_id = path[i + 1]
            next_node = self.maze_map.nodes[next_id]
            
            dx = next_node.x - current_node.x
            dy = next_node.y - current_node.y
            
            if abs(dx) > abs(dy):
                target_direction = 'x+' if dx > 0 else 'x-'
            else:
                target_direction = 'y+' if dy > 0 else 'y-'
            
            # **รวมระยะทางที่เป็นทิศเดียวกัน**
            total_distance = 0
            j = i
            segments = []
            
            while j < len(path) - 1:
                current_segment_node = self.maze_map.nodes[path[j]]
                next_segment_node = self.maze_map.nodes[path[j + 1]]
                
                # คำนวณทิศทางของ segment นี้
                seg_dx = next_segment_node.x - current_segment_node.x
                seg_dy = next_segment_node.y - current_segment_node.y
                segment_distance = math.sqrt(seg_dx*seg_dx + seg_dy*seg_dy)
                
                if abs(seg_dx) > abs(seg_dy):
                    segment_direction = 'x+' if seg_dx > 0 else 'x-'
                else:
                    segment_direction = 'y+' if seg_dy > 0 else 'y-'
                
                # ถ้าทิศทางเดียวกัน ให้รวมระยะทาง
                if segment_direction == target_direction:
                    total_distance += segment_distance
                    segments.append(f"{path[j]} -> {path[j+1]} ({segment_distance:.2f}m)")
                    j += 1
                else:
                    break
            
            # หันไปทิศทางที่ต้องการ
            self.turn_to_direction(target_direction)
            
            # เดินระยะทางรวม
            if len(segments) > 1:
                print(f"🚶 เดินยาว {total_distance:.2f}m ทิศทาง {target_direction}:")
                for seg in segments:
                    print(f"   - {seg}")
            else:
                print(f"🚶 เดิน {total_distance:.2f}m ทิศทาง {target_direction}")
            
            move_forward(total_distance)
            
            # เดินต่อจากจุดสุดท้าย
            i = j

    def turn_to_direction(self, target_direction):
        """หันไปทิศทางที่ต้องการ - หมุน 180° ในครั้งเดียว"""
        current_facing = self.current_facing_direction
        
        if target_direction == current_facing:
            return
        
        right_sequence = ['x+', 'y+', 'x-', 'y-']  # หมุนขวา: ตะวันออก → เหนือ → ตะวันตก → ใต้

        current_index = right_sequence.index(current_facing)
        target_index = right_sequence.index(target_direction)
        
        # คำนวณการหมุนที่สั้นที่สุด
        diff = (target_index - current_index) % 4
        
        if diff == 1:  # หมุนขวา 90°
            print(f"🔄 หมุนขวา 90°")
            turn_right(90)
            self.update_facing_direction('right', 90)
        elif diff == 2:  # หมุน 180°
            print(f"🔄 หมุน 180°")
            turn_right(180)  # **หมุน 180° ในครั้งเดียว**
            self.update_facing_direction('right', 180)
        elif diff == 3:  # หมุนซ้าย 90°
            print(f"🔄 หมุนซ้าย 90°")
            turn_left(90)
            self.update_facing_direction('left', 90)
        
        time.sleep(0.2)

    # ...existing code...

# ...existing code...
    
    def show_map(self):
        """แสดงแผนที่พร้อมข้อมูลทางตันแบบละเอียด"""
        print(f"\n📋 แผนที่: {len(self.maze_map.nodes)} โหนด")
        for node_id, node in self.maze_map.nodes.items():
            connections = [f"{n}({d:.1f})" for n, d in node.connections.items()]
            blocked_info = f" 🚫{list(node.blocked_directions)}" if node.blocked_directions else ""
            
            print(f"   โหนด {node_id} ({node.x:.3f}, {node.y:.3f}): {connections}{blocked_info}")
    
    def return_home(self, start_node, end_node):
        """กลับบ้าน (พิจารณาทางตัน)"""
        print(f"\n🔍 หาเส้นทางกลับ: {end_node} -> {start_node}")
        
        path, distance = a_star_search(self.maze_map, end_node, start_node)
        
        if not path:
            print("❌ ไม่เจอทาง!")
            return None, 0
            
        print(f"✅ เจอแล้ว! {distance:.1f}m")
        print(f"🗺️ เส้นทาง: {' -> '.join(map(str, path))}")
        
        # **เปลี่ยนจาก simple_walk_path เป็น smart_walk_path**
        self.smart_walk_path(path)
        
        return path, distance
    
    def smart_walk_path(self, path):
        """เดินกลับบ้าน - รวมระยะทางที่เป็นทิศเดียวกัน"""
        print("\n🚶 เริ่มเดินกลับ (เดินยาวๆ ต่อเนื่อง)")
        
        i = 0
        while i < len(path) - 1:
            current_node = self.maze_map.nodes[path[i]]
            next_node = self.maze_map.nodes[path[i + 1]]
            
            # หาทิศทางแรก
            dx = next_node.x - current_node.x
            dy = next_node.y - current_node.y
            
            if abs(dx) > abs(dy):
                target_direction = 'x+' if dx > 0 else 'x-'
            else:
                target_direction = 'y+' if dy > 0 else 'y-'
            
            # **รวมระยะทางที่เป็นทิศเดียวกัน**
            total_distance = 0
            j = i
            segments = []
            
            while j < len(path) - 1:
                current_seg_node = self.maze_map.nodes[path[j]]
                next_seg_node = self.maze_map.nodes[path[j + 1]]
                
                seg_dx = next_seg_node.x - current_seg_node.x
                seg_dy = next_seg_node.y - current_seg_node.y
                segment_distance = math.sqrt(seg_dx*seg_dx + seg_dy*seg_dy)
                
                if abs(seg_dx) > abs(seg_dy):
                    segment_direction = 'x+' if seg_dx > 0 else 'x-'
                else:
                    segment_direction = 'y+' if seg_dy > 0 else 'y-'
                
                # ถ้าทิศทางเดียวกัน ให้รวมระยะทาง
                if segment_direction == target_direction:
                    total_distance += segment_distance
                    segments.append(f"โหนด {path[j]} -> {path[j+1]}")
                    j += 1
                else:
                    break
        
        # หันไปทิศทางที่ต้องการ
        self.turn_to_direction(target_direction)
        
        # เดินระยะทางรวม
        if len(segments) > 1:
            print(f"🚶 เดินยาว {total_distance:.3f}m ทิศทาง {target_direction}:")
            for seg in segments:
                print(f"   - {seg}")
        else:
            print(f"🚶 เดิน {total_distance:.3f}m: {segments[0]}")
        
        move_forward(total_distance)
        
        # เดินต่อจากจุดสุดท้าย
        i = j
    
        print("🏠 ถึงบ้านแล้ว!")

    def manual_calibrate_orientation(self):
        """
        Calibrate ทิศทางแบบ manual โดยดูการเคลื่อนที่จริง
        """
        print("\n🧭 กำลัง calibrate ทิศทางแบบ manual...")
        
        # บันทึกตำแหน่งเริ่มต้น
        start_pos = get_current_position()
        start_yaw = get_current_orientation()
        
        print(f"📍 ตำแหน่งเริ่มต้น: ({start_pos[0]:.3f}, {start_pos[1]:.3f})")
        print(f"🧭 มุมเริ่มต้น: {start_yaw:.1f}°")
        
        # เดินทดสอบ
        print("🚶 เดินทดสอบเพื่อ calibrate...")
        move_forward(0.2)  # เดิน 20cm
        
        # ตรวจสอบการเปลี่ยนแปลง
        end_pos = get_current_position()
        dx = end_pos[0] - start_pos[0]
        dy = end_pos[1] - start_pos[1]
        
        print(f"📍 ตำแหน่งหลังเดิน: ({end_pos[0]:.3f}, {end_pos[1]:.3f})")
        print(f"📐 การเปลี่ยนแปลง: dx={dx:.3f}, dy={dy:.3f}")
        
        # กำหนดทิศทางจริงจากการเคลื่อนที่
        if abs(dx) > abs(dy):
            if dx > 0:
                real_direction = 'x+'  # เคลื่อนที่ไปทิศตะวันออก
                print("🧭 หุ่นยนต์หันหน้าไปทิศตะวันออก (x+)")
            else:
                real_direction = 'x-'  # เคลื่อนที่ไปทิศตะวันตก
                print("🧭 หุ่นยนต์หันหน้าไปทิศตะวันตก (x-)")
        else:
            if dy > 0:
                real_direction = 'y+'  # เคลื่อนที่ไปทิศเหนือ
                print("🧭 หุ่นยนต์หันหน้าไปทิศเหนือ (y+)")
            else:
                real_direction = 'y-'  # เคลื่อนที่ไปทิศใต้
                print("🧭 หุ่นยนต์หันหน้าไปทิศใต้ (y-)")
        
        # Calibrate ด้วยทิศทางที่ตรวจพบ
        calibrate_initial_orientation(real_direction)
        
        # กลับตำแหน่งเดิม
        print("🔙 กลับตำแหน่งเดิม...")
        move_forward(-0.2)  # ถอยหลัง
        
        print("✅ Manual calibration เสร็จสิ้น!")

def main():
    print("🤖 เริ่มระบบ...")
    
    # เริ่มต้นระบบการเคลื่อนที่
    init_movement_system()
    
    runner = MazeRunner()
    
    # try:
    print("⏳ รอ 2 วินาที...")
    time.sleep(0.5)
    
    # สำรวจ
    start, end = runner.explore_maze()
    
    print("\n⏱️ พัก 2 วินาที...")
    time.sleep(0.5)
    
    # กลับ
    path, distance = runner.return_home(start, end)
    
    print(f"\n🎉 สำเร็จ!")
    print(f"📏 ระยะทางกลับ: {distance:.1f}m")
    print(f"🗺️ โหนดทั้งหมด: {len(runner.maze_map.nodes)}")
        
    # except KeyboardInterrupt:
    #     print("\n⏹️ หยุด")
    # except Exception as e:
    #     print(f"\n❌ ผิดพลาด: {e}")
    # finally:
    cleanup_movement_system()

if __name__ == '__main__':
    main()