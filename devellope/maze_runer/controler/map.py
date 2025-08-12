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
        self.blocked_directions = []  # เก็บข้อมูลสแกน [y-, x+, y+, x-]
        self.marker = []
    
    def add_blocked_direction(self, direction):
        """บันทึกข้อมูลสแกนตรงๆ [0,0,0,0]"""
        if direction:
            self.blocked_directions = direction  # บันทึกตรงๆ
    
    def __repr__(self):
        blocked_str = f", scan_data={self.blocked_directions}" if self.blocked_directions else ""
        return f"Node(id={self.id}, pos=({self.x},{self.y}), exit={self.is_exit}{blocked_str})"

class Graph:
    def __init__(self):
        self.nodes = {}
        self.node_count = 0

    def add_node(self, x, y, is_exit=False):
        """เพิ่มโหนด"""
        new_id = self.node_count
        new_node = Node(new_id, x, y, is_exit)
        self.nodes[new_id] = new_node
        self.node_count += 1
        print(f"📍 สร้างโหนดใหม่: Node(id={new_id}, pos=({x:.2f},{y:.2f}))")
        return new_id
    
    def add_blocked_direction_to_node(self, node_id, direction):
        """บันทึกข้อมูลสแกนลงโหนด"""
        if node_id in self.nodes:
            self.nodes[node_id].add_blocked_direction(direction)
            print(f"🗺️  บันทึกข้อมูลสแกน: โหนด {node_id} = {direction}")
        else:
            print(f"❌ ไม่พบโหนด {node_id}")

    def add_edge(self, node1_id, node2_id, distance):
        """เชื่อมโหนดสองโหนด"""
        if node1_id in self.nodes and node2_id in self.nodes:
            # ตรวจสอบว่าเชื่อมแล้วหรือยัง
            if node2_id not in self.nodes[node1_id].connections:
                self.nodes[node1_id].connections[node2_id] = distance
                self.nodes[node2_id].connections[node1_id] = distance
                print(f"🔗 เชื่อมเส้นทาง: โหนด {node1_id} <-> {node2_id} (ระยะทาง: {distance:.3f})")
            else:
                print(f"🔍 เส้นทางมีอยู่แล้ว: โหนด {node1_id} <-> {node2_id}")
        else:
            print(f"❌ ไม่สามารถเชื่อมโหนด {node1_id} และ {node2_id} ได้")

def calculate_direction(from_node, to_node):
    """คำนวณทิศทางจากโหนดหนึ่งไปอีกโหนดหนึ่ง"""
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    
    if abs(dx) > abs(dy):
        return 'x+' if dx > 0 else 'x-'
    else:
        return 'y+' if dy > 0 else 'y-'

def heuristic(node1, node2):
    """คำนวณระยะทางแบบยุคลิด"""
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def a_star_search(graph, start_id, goal_id):
    """A* search algorithm"""
    if start_id not in graph.nodes or goal_id not in graph.nodes:
        print(f"❌ โหนดเริ่มต้น ({start_id}) หรือโหนดเป้าหมาย ({goal_id}) ไม่พบ")
        return None, 0
    
    start_node = graph.nodes[start_id]
    goal_node = graph.nodes[goal_id]
    
    open_set = [(0, start_id)]
    came_from = {}
    g_score = {start_id: 0}
    f_score = {start_id: heuristic(start_node, goal_node)}
    
    print(f"🔍 A* Search: {start_id} -> {goal_id}")
    
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
            tentative_g_score = g_score[current_id] + edge_distance
            
            if neighbor_id not in g_score or tentative_g_score < g_score[neighbor_id]:
                came_from[neighbor_id] = current_id
                g_score[neighbor_id] = tentative_g_score
                f_score[neighbor_id] = tentative_g_score + heuristic(graph.nodes[neighbor_id], goal_node)
                
                if (f_score[neighbor_id], neighbor_id) not in open_set:
                    heapq.heappush(open_set, (f_score[neighbor_id], neighbor_id))
    
    print(f"❌ ไม่พบเส้นทางจาก {start_id} ไป {goal_id}")
    return None, 0