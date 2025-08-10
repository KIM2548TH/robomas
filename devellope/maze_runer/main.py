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
    detect_walls_at_current_position,
    convert_blocked_directions_to_compass
)
from controler.map import Graph, a_star_search, Node, calculate_direction

class MazeRunner:
    def __init__(self):
        self.maze_map = Graph()
        self.current_node_id = None
        self.path_history = []
        self.move_distance = 0.5
        set_move_distance(self.move_distance)
        
    def get_real_position(self):
        """ดึงตำแหน่งจริงจากเซ็นเซอร์"""
        return get_current_position()
        
    def add_node_here(self, is_exit=False, blocked_directions=None, auto_detect_walls=True):
        """
        เพิ่มโหนดที่ตำแหน่งปัจจุบัน พร้อมตรวจจับทางตันอัตโนมัติ
        
        Args:
            is_exit (bool): เป็นทางออกหรือไม่
            blocked_directions (list): รายการทิศทางที่เป็นทางตัน (manual)
            auto_detect_walls (bool): ตรวจจับทางตันอัตโนมัติหรือไม่
        """
        x, y = self.get_real_position()
        
        print(f"📍 ตำแหน่งจากเซ็นเซอร์: x={x:.3f}, y={y:.3f}")
        
        # ตรวจจับทางตันอัตโนมัติด้วย subscription
        detected_walls = []
        if auto_detect_walls:
            print("🔍 เริ่มตรวจจับทางตันด้วย subscription...")
            detected_walls = detect_walls_at_current_position()
            
        # รวมทางตันที่ตรวจพบกับที่ระบุ manual
        all_blocked = []
        if detected_walls:
            compass_blocked = convert_blocked_directions_to_compass(detected_walls)
            all_blocked.extend(compass_blocked)
            print(f"🤖 ตรวจพบทางตันอัตโนมัติ: {detected_walls} → {compass_blocked}")
            
        if blocked_directions:
            all_blocked.extend(blocked_directions)
            print(f"👤 ระบุทางตัน manual: {blocked_directions}")
        
        # ลบรายการซ้ำ
        final_blocked = list(set(all_blocked))
        
        # แสดงข้อมูลทางตันรวม
        if final_blocked:
            print(f"🚫 ทางตันรวม: {final_blocked}")
        
        # แสดงสถิติการเคลื่อนที่
        stats = get_movement_stats()
        print(f"🚀 ความเร็ว: {stats['speed']:.3f}m/s")
        print(f"⚖️ ความเสถียร: {stats['stable_count']} iterations")
        
        display_x = round(x, 1)
        display_y = round(y, 1)
        
        node_id = self.maze_map.add_node(x=x, y=y, is_exit=is_exit, 
                                       blocked_directions=final_blocked)
        print(f"📍 เพิ่มโหนด {node_id} ที่ ({display_x}, {display_y})")
        return node_id

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
    
    def move_with_detection(self, turn_direction=None, turn_angle=90, 
                          blocked_directions=None, auto_detect_walls=True):
        """
        เดินแบบตรวจสอบโหนดเดิมเสมอ พร้อมตรวจจับทางตัน
        
        Args:
            turn_direction (str): ทิศทางการเลี้ยว 'right' หรือ 'left'
            turn_angle (float): มุมการเลี้ยว (องศา)
            blocked_directions (list): ทิศทางที่เป็นทางตันสำหรับโหนดใหม่ (manual)
            auto_detect_walls (bool): ตรวจจับทางตันอัตโนมัติหรือไม่
        """
        # เลี้ยว (ถ้าต้องการ)
        if turn_direction == "right":
            print(f"🔄 เลี้ยวขวา {turn_angle}°")
            turn_right(turn_angle)
        elif turn_direction == "left":
            print(f"🔄 เลี้ยวซ้าย {turn_angle}°")
            turn_left(turn_angle)
            
        time.sleep(0.2)
        
        # เดิน
        print(f"🚶 เดิน {self.move_distance}m")
        move_forward(self.move_distance)
        
        # เช็คโหนดเดิมเสมอ
        existing_node = self.find_existing_node()
        
        if existing_node is not None:
            # ใช้โหนดเดิม
            print(f"✅ ใช้โหนดเดิม {existing_node}")
            node_id = existing_node
            
            # เพิ่มทางตันให้โหนดเดิม (ถ้ามี)
            if auto_detect_walls or blocked_directions:
                print("🔍 ตรวจจับทางตันเพิ่มเติมสำหรับโหนดเดิม...")
                if auto_detect_walls:
                    detected_walls = detect_walls_at_current_position()
                    compass_blocked = convert_blocked_directions_to_compass(detected_walls)
                    for direction in compass_blocked:
                        self.maze_map.add_blocked_direction_to_node(node_id, direction)
                
                if blocked_directions:
                    for direction in blocked_directions:
                        self.maze_map.add_blocked_direction_to_node(node_id, direction)
        else:
            # สร้างโหนดใหม่
            print(f"➕ สร้างโหนดใหม่")
            node_id = self.add_node_here(blocked_directions=blocked_directions,
                                       auto_detect_walls=auto_detect_walls)
        
        # เชื่อมกับโหนดก่อนหน้า
        if self.path_history:
            prev_node = self.path_history[-1]
            if prev_node != node_id:  # ไม่เชื่อมกับตัวเอง
                if prev_node not in self.maze_map.nodes[node_id].connections:
                    self.maze_map.add_edge(prev_node, node_id, self.move_distance)
                    print(f"🔗 เชื่อม {prev_node} -> {node_id}")
                else:
                    print(f"✓ เชื่อมแล้ว {prev_node} -> {node_id}")
        
        self.path_history.append(node_id)
        self.current_node_id = node_id
        
        return node_id
    
    def explore_maze(self):
        """สำรวจเขาวงกต พร้อมตรวจจับทางตันอัตโนมัติ"""
        print("🏁 เริ่มสำรวจเขาวงกต!")
        
        # จุดเริ่มต้น - ตรวจจับทางตันอัตโนมัติ
        start_node = self.add_node_here(auto_detect_walls=True)
        self.path_history.append(start_node)
        self.current_node_id = start_node
        
        # 1. เดิน 3 ครั้ง
        print("\n1️⃣ เดิน 3 ครั้ง")
        for i in range(3):
            print(f"\n--- ครั้งที่ {i+1} ---")
            self.move_with_detection(auto_detect_walls=True)
            time.sleep(0.3)
        
        # 2. เลี้ยวขวา + เดิน
        print("\n2️⃣ เลี้ยวขวา + เดิน")
        self.move_with_detection("right", 90, auto_detect_walls=True)
        time.sleep(0.3)
        
        # 3. เลี้ยวขวา + เดิน
        print("\n3️⃣ เลี้ยวขวา + เดิน")
        self.move_with_detection("right", 90, auto_detect_walls=True)
        time.sleep(0.3)
        
        # 4. เลี้ยวขวา + เดิน (ควรเจอโหนดเดิม)
        print("\n4️⃣ เลี้ยวขวา + เดิน")
        self.move_with_detection("right", 90, auto_detect_walls=True)
        time.sleep(0.3)
        
        # 5. เดิน 2 ครั้ง
        print("\n5️⃣ เดิน 2 ครั้ง")
        for i in range(2):
            print(f"\n--- ครั้งที่ {i+1} ---")
            self.move_with_detection(auto_detect_walls=True)
            time.sleep(0.3)
        
        # เลี้ยวซ้าย 2 ครั้ง + เดิน
        print("\n6️⃣ เลี้ยวซ้าย + เดิน")
        self.move_with_detection("left", 90, auto_detect_walls=True)
        time.sleep(0.3)
        
        print("\n7️⃣ เลี้ยวซ้าย + เดิน")
        self.move_with_detection("left", 90, auto_detect_walls=True)
        time.sleep(0.3)
        
        print("\n8️⃣ เดินสุดท้าย")
        self.move_with_detection(auto_detect_walls=True)
        time.sleep(0.3)
        
        # กำหนดทางออก
        end_node = self.current_node_id
        self.maze_map.nodes[end_node].is_exit = True
        print(f"🎯 โหนด {end_node} = ทางออก")
        
        # แสดงแผนที่
        self.show_map()
        
        return start_node, end_node
    
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
        
        # เดินกลับ
        self.walk_path(path)
        
        return path, distance
    
    def walk_path(self, path):
        """เดินตามเส้นทาง"""
        print("\n🚶 เริ่มเดินกลับ")
        
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
            
            # คำนวณทิศทางและหมุน
            direction = calculate_direction(current_node, next_node)
            current_yaw = get_current_orientation()
            
            # คำนวณมุมเป้าหมาย
            direction_angles = {'north': 0, 'east': 90, 'south': 180, 'west': -90}
            target_angle = direction_angles.get(direction, 0)
            
            # คำนวณมุมที่ต้องหมุน
            angle_diff = target_angle - current_yaw
            while angle_diff > 180:
                angle_diff -= 360
            while angle_diff < -180:
                angle_diff += 360
            
            print(f"🧭 ทิศทาง: {direction}, ต้องหมุน: {angle_diff:.1f}°")
            
            # หมุนถ้าจำเป็น
            if abs(angle_diff) > 5:
                if angle_diff > 0:
                    turn_left(abs(angle_diff))
                else:
                    turn_right(abs(angle_diff))
            
            time.sleep(0.3)
            
            # เดิน
            move_forward(distance)
            time.sleep(0.3)
        
        print("🏠 ถึงบ้านแล้ว!")

def main():
    print("🤖 เริ่มระบบ...")
    
    # เริ่มต้นระบบการเคลื่อนที่
    init_movement_system()
    
    runner = MazeRunner()
    
    try:
        print("⏳ รอ 2 วินาที...")
        time.sleep(2)
        
        # สำรวจ
        start, end = runner.explore_maze()
        
        print("\n⏱️ พัก 2 วินาที...")
        time.sleep(2)
        
        # กลับ
        path, distance = runner.return_home(start, end)
        
        print(f"\n🎉 สำเร็จ!")
        print(f"📏 ระยะทางกลับ: {distance:.1f}m")
        print(f"🗺️ โหนดทั้งหมด: {len(runner.maze_map.nodes)}")
        
    except KeyboardInterrupt:
        print("\n⏹️ หยุด")
    except Exception as e:
        print(f"\n❌ ผิดพลาด: {e}")
    finally:
        cleanup_movement_system()

if __name__ == '__main__':
    main()