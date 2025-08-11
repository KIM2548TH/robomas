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
    get_real_direction_from_yaw,
    get_target_yaw_for_direction
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
        self.current_facing_direction = 'x+'  # เริ่มต้นหันหน้าทิศเหนือ
        self.direction_history = []  # บันทึกประวัติการหมุน
        
        print(f"🧭 เริ่มต้นหันหน้าไปทิศ: {self.current_facing_direction} (เหนือ)")
    
    def update_facing_direction(self, turn_direction, angle=90):
        """
        อัปเดตทิศทางหลังจากหมุน
        
        Args:
            turn_direction (str): 'right' หรือ 'left'
            angle (float): มุมที่หมุน (องศา)
        """
        if angle != 90:
            print(f"⚠️ รองรับเฉพาะการหมุน 90° เท่านั้น (ได้รับ {angle}°)")
            return
        
        # ลำดับทิศทางสำหรับการหมุนขวา
        right_sequence = ['y+', 'x+', 'y-', 'x-']  # เหนือ → ตะวันออก → ใต้ → ตะวันตก
        
        current_index = right_sequence.index(self.current_facing_direction)
        
        if turn_direction.lower() == 'right':
            new_index = (current_index + 1) % 4
        elif turn_direction.lower() == 'left':
            new_index = (current_index - 1) % 4
        else:
            print(f"❌ ทิศทางการหมุนไม่ถูกต้อง: {turn_direction}")
            return
        
        old_direction = self.current_facing_direction
        self.current_facing_direction = right_sequence[new_index]
        
        # บันทึกประวัติ
        self.direction_history.append({
            'action': f'turn_{turn_direction}',
            'from': old_direction,
            'to': self.current_facing_direction,
            'node': self.current_node_id
        })
        
        direction_names = {
            'y+': 'เหนือ', 'y-': 'ใต้', 'x+': 'ตะวันออก', 'x-': 'ตะวันตก'
        }
        
        print(f"✅ หมุน{turn_direction}: {direction_names[old_direction]} → {direction_names[self.current_facing_direction]}")

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
        max_steps = 20
        # visited_nodes = {start_node}
        print(wall,656565656565)
        
        for step in range(1, max_steps + 1):
            print(f"\n--- ขั้นตอนที่ {step} ---")
            
            # ✅ 1. เช็คโหนดปัจจุบันว่ามีทางไหนไปได้บ้าง (ที่เราเช็คด้วยกิมบอลแล้ว)
            current_node = self.maze_map.nodes[self.current_node_id]
            print(9999)
            # wall = list(current_node.blocked_directions)
            
            print(f"📍 ตรวจสอบโหนดปัจจุบัน {self.current_node_id}:")
            print(f"   - ทางตัน (เช็คด้วยกิมบอลแล้ว): {wall}")
            

            # ✅ 3. เลือกทิศทางแรกที่ยังไม่เคยไป (จากที่เช็คแล้ว)
            chosen_direction = ''
            if wall :
                if wall[0] == 1:
                    chosen_direction = 'right'  # เลือกซ้าย
                elif wall[1] == 1:
                    chosen_direction = 'front'  # เลือกหน้า
                elif wall[2] == 1:
                    chosen_direction = 'left'  # เลือกขวา
                elif wall[0] + wall[1] + wall[2] == 0:
                    chosen_direction = "back"
                print(f"🎯 เลือกไปทิศทาง: {chosen_direction} (เช็คด้วยกิมบอลแล้ว)")
            
 
            print(f"🎯 ต้องหันไป: {chosen_direction}")


            if chosen_direction == 'left':
                print("➡️ หันหน้าไปทิศซ้าย")
                turn_left(90)
                time.sleep(0.1)
                move_forward(0.6)
            elif chosen_direction == 'right':
                print("➡️ หันหน้าไปทิศขวา")
                turn_right(90)
                time.sleep(0.1)
                move_forward(0.6)
            elif chosen_direction == 'front':
                print("➡️ หันหน้าไปทิศหน้า")
                move_forward(0.6)
            elif chosen_direction == 'back':
                print("➡️ หันหน้าไปทิศหลัง")
                turn_left(180)
                time.sleep(0.1)
                move_forward(0.6)

            # ✅ 5. เดินไปทิศทางที่เลือก
            print(f"🚶 เดินไปทิศทาง {chosen_direction}")

            new_node_id,wall = self.add_node_here(auto_detect_walls=True)
            new_node = self.maze_map.nodes[self.current_node_id]

            self.maze_map.add_edge(self.current_node_id, new_node_id, 0.6)
            self.current_node_id = new_node_id


            self.path_history.append(new_node)
            print(self.path_history,8888)
            print(888)
            
            # self.current_node_id
            # visited_nodes.add(new_node)
            
            print(f"📊 สถิติ: โหนด {len(self.maze_map.nodes)}, ")
            time.sleep(0.5)
        
        # หาโหนดที่ไกลที่สุดเป็นทางออก
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
        """เดินตามเส้นทาง - ติดตามทิศทางปัจจุบันของหุ่นยนต์อย่างถูกต้อง"""
        print("\n🚶 เริ่มเดินกลับ (ติดตามทิศทางจริง)")
        
        # **คำนวณทิศทางเริ่มต้นจากการสำรวจ**
        # สำรวจ: เริ่มต้น y+ → เดิน 3 ครั้ง → หมุนขวา 3 ครั้ง → เดิน 2 ครั้ง → หมุนซ้าย 2 ครั้ง
        # ทิศทางสุดท้าย: y+ → (หมุนขวา 3 ครั้ง) → y- → (หมุนซ้าย 2 ครั้ง) → y-
        current_direction = 'y-'  # หันหน้าไปทิศใต้
        
        print(f"🧭 เริ่มต้นหันหน้าไปทิศ: {current_direction} (ใต้)")
        
        # แมปทิศทางและการหมุน
        direction_names = {
            'y+': 'เหนือ', 'y-': 'ใต้', 'x+': 'ตะวันออก', 'x-': 'ตะวันตก'
        }
        
        # การหมุนจากทิศทางปัจจุบันไปทิศทางเป้าหมาย (หมุนขวา 90° เท่านั้น)
        right_turn_sequence = ['y+', 'x+', 'y-', 'x-']  # ลำดับการหมุนขวา
        
        for i in range(len(path) - 1):
            current_node = self.maze_map.nodes[path[i]]
            next_node = self.maze_map.nodes[path[i + 1]]
            
            print(f"\n🚶‍♂️ โหนด {path[i]} -> โหนด {path[i+1]}")
            
            # คำนวณระยะทางและทิศทาง
            dx = next_node.x - current_node.x
            dy = next_node.y - current_node.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            print(f"📍 จาก ({current_node.x:.3f}, {current_node.y:.3f}) "
                  f"ไป ({next_node.x:.3f}, {next_node.y:.3f})")
            print(f"📏 ระยะทาง: {distance:.3f}m")
            print(f"📐 การเปลี่ยนแปลง: dx={dx:.3f}, dy={dy:.3f}")
            
            # คำนวณทิศทางที่ต้องการ
            if abs(dx) > abs(dy):
                target_direction = 'x+' if dx > 0 else 'x-'
            else:
                target_direction = 'y+' if dy > 0 else 'y-'
            
            print(f"🧭 ปัจจุบันหันหน้าไปทิศ: {current_direction} ({direction_names[current_direction]})")
            print(f"🎯 ต้องการไปทิศ: {target_direction} ({direction_names[target_direction]})")
            
            # คำนวณจำนวนครั้งที่ต้องหมุนขวา
            current_index = right_turn_sequence.index(current_direction)
            target_index = right_turn_sequence.index(target_direction)
            
            # คำนวณจำนวนครั้งที่ต้องหมุนขวา
            if target_index >= current_index:
                turns_needed = target_index - current_index
            else:
                turns_needed = (4 - current_index) + target_index
            
            print(f"🔄 ต้องหมุนขวา {turns_needed} ครั้ง")
            
            # หมุน
            for turn_num in range(turns_needed):
                print(f"   🔄 หมุนขวา 90° (ครั้งที่ {turn_num + 1})")
                turn_right(90)
                time.sleep(0.2)
            
            # อัปเดตทิศทางปัจจุบัน
            if turns_needed > 0:
                current_direction = target_direction
                print(f"✅ ตอนนี้หันหน้าไปทิศ: {current_direction} ({direction_names[current_direction]})")
            else:
                print("➡️ ไม่ต้องหมุน (ทิศทางถูกต้องแล้ว)")
            
            time.sleep(0.3)
            
            # เดิน
            print(f"🚶 เดิน {distance:.3f}m ในทิศทาง {target_direction}")
            move_forward(distance)
            time.sleep(0.3)
            
            # ตรวจสอบตำแหน่งหลังเดิน
            final_pos = get_current_position()
            print(f"📍 ตำแหน่งหลังเดิน: ({final_pos[0]:.3f}, {final_pos[1]:.3f})")
            
            # ตรวจสอบว่าเดินถูกทิศทางหรือไม่
            end_dx = final_pos[0] - current_node.x
            end_dy = final_pos[1] - current_node.y
            
            if target_direction == 'x+' and end_dx > 0:
                print("✅ เดินถูกทิศทาง (ไปทิศตะวันออก)")
            elif target_direction == 'x-' and end_dx < 0:
                print("✅ เดินถูกทิศทาง (ไปทิศตะวันตก)")
            elif target_direction == 'y+' and end_dy > 0:
                print("✅ เดินถูกทิศทาง (ไปทิศเหนือ)")
            elif target_direction == 'y-' and end_dy < 0:
                print("✅ เดินถูกทิศทาง (ไปทิศใต้)")
            else:
                print("⚠️ เดินผิดทิศทาง!")
    
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