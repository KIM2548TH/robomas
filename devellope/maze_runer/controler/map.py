# -*-coding:utf-8-*-
import math
import heapq

class Node:
    def __init__(self, node_id, x, y, is_exit=False):
        self.id = node_id
        self.x = x
        self.y = y
        self.is_exit = is_exit
        self.connections = {}  # {neighbor_id: distance}
        
        # **เพิ่ม: ระบบระบุทางตัน**
        self.blocked_directions = set()  # เก็บทิศทางที่เป็นทางตัน
        # ทิศทาง: 'north', 'south', 'east', 'west'
    
    def add_blocked_direction(self, direction):
        """เพิ่มทิศทางที่เป็นทางตัน"""
        valid_directions = {'north', 'south', 'east', 'west'}
        if direction.lower() in valid_directions:
            self.blocked_directions.add(direction.lower())
            print(f"🚫 โหนด {self.id}: เพิ่มทางตัน {direction}")
    
    def remove_blocked_direction(self, direction):
        """ลบทิศทางทางตัน"""
        if direction.lower() in self.blocked_directions:
            self.blocked_directions.remove(direction.lower())
            print(f"✅ โหนด {self.id}: ลบทางตัน {direction}")
    
    def is_blocked_direction(self, direction):
        """ตรวจสอบว่าทิศทางนั้นเป็นทางตันหรือไม่"""
        return direction.lower() in self.blocked_directions
    
    def get_available_directions(self):
        """ดึงทิศทางที่สามารถไปได้"""
        all_directions = {'north', 'south', 'east', 'west'}
        return all_directions - self.blocked_directions
    
    def __repr__(self):
        blocked_str = f", blocked={list(self.blocked_directions)}" if self.blocked_directions else ""
        return f"Node(id={self.id}, pos=({self.x},{self.y}), exit={self.is_exit}{blocked_str})"

class Graph:
    def __init__(self):
        self.nodes = {}
        self.node_count = 0

    def add_node(self, x, y, is_exit=False, blocked_directions=None):
        """เพิ่มโหนดพร้อมระบุทางตัน"""
        new_id = self.node_count
        new_node = Node(new_id, x, y, is_exit)
        
        # เพิ่มทางตันถ้ามี
        if blocked_directions:
            for direction in blocked_directions:
                new_node.add_blocked_direction(direction)
        
        self.nodes[new_id] = new_node
        self.node_count += 1
        print(f"📍 พบโหนดใหม่: {new_node}")
        return new_id
    
    def add_blocked_direction_to_node(self, node_id, direction):
        """เพิ่มทางตันให้โหนดที่มีอยู่แล้ว"""
        if node_id in self.nodes:
            self.nodes[node_id].add_blocked_direction(direction)
        else:
            print(f"❌ ไม่พบโหนด {node_id}")

    def add_edge(self, node1_id, node2_id, distance):
        """เชื่อมโหนดสองโหนด"""
        if node1_id in self.nodes and node2_id in self.nodes:
            self.nodes[node1_id].connections[node2_id] = distance
            self.nodes[node2_id].connections[node1_id] = distance
            print(f"🔗 สร้างเส้นทางระหว่างโหนด {node1_id} <-> {node2_id} (ระยะทาง: {distance})")
        else:
            print(f"❌ ไม่สามารถเชื่อมโหนด {node1_id} และ {node2_id} ได้")

def calculate_direction(from_node, to_node):
    """คำนวณทิศทางจากโหนดหนึ่งไปอีกโหนดหนึ่ง"""
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    
    # ตรวจสอบทิศทางหลัก
    if abs(dx) > abs(dy):
        return 'east' if dx > 0 else 'west'
    else:
        return 'north' if dy > 0 else 'south'

def heuristic(node1, node2):
    """คำนวณระยะทางแบบยุคลิด"""
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def a_star_search(graph, start_id, goal_id):
    """A* search algorithm พร้อมพิจารณาทางตัน"""
    if start_id not in graph.nodes or goal_id not in graph.nodes:
        print(f"❌ โหนดเริ่มต้น ({start_id}) หรือโหนดเป้าหมาย ({goal_id}) ไม่พบ")
        return None, 0
    
    start_node = graph.nodes[start_id]
    goal_node = graph.nodes[goal_id]
    
    open_set = [(0, start_id)]
    came_from = {}
    g_score = {start_id: 0}
    f_score = {start_id: heuristic(start_node, goal_node)}
    
    while open_set:
        current_f, current_id = heapq.heappop(open_set)
        
        if current_id == goal_id:
            # สร้างเส้นทาง
            path = []
            total_distance = g_score[goal_id]
            
            while current_id in came_from:
                path.append(current_id)
                current_id = came_from[current_id]
            path.append(start_id)
            path.reverse()
            
            return path, total_distance
        
        current_node = graph.nodes[current_id]
        
        for neighbor_id, edge_distance in current_node.connections.items():
            neighbor_node = graph.nodes[neighbor_id]
            
            # **ตรวจสอบทางตัน**
            direction_to_neighbor = calculate_direction(current_node, neighbor_node)
            if current_node.is_blocked_direction(direction_to_neighbor):
                print(f"🚫 ข้าม: {current_id} -> {neighbor_id} (ทิศ {direction_to_neighbor} เป็นทางตัน)")
                continue
            
            tentative_g_score = g_score[current_id] + edge_distance
            
            if neighbor_id not in g_score or tentative_g_score < g_score[neighbor_id]:
                came_from[neighbor_id] = current_id
                g_score[neighbor_id] = tentative_g_score
                f_score[neighbor_id] = tentative_g_score + heuristic(neighbor_node, goal_node)
                
                if (f_score[neighbor_id], neighbor_id) not in open_set:
                    heapq.heappush(open_set, (f_score[neighbor_id], neighbor_id))
    
    print(f"❌ ไม่พบเส้นทางจาก {start_id} ไป {goal_id}")
    return None, 0