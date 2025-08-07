import heapq
import math

# =======================================================
# == ส่วนที่ 1: คลาส Node และ Graph (โค้ดเดิมของคุณ) ==
# =======================================================
class Node:
    def __init__(self, node_id, x, y, is_exit=False):
        self.id = node_id          # ID ของโหนด (เช่น 0, 1, 2)
        self.x = x                 # พิกัด X (สำหรับคำนวณ Heuristic)
        self.y = y                 # พิกัด Y
        self.is_exit = is_exit     # True ถ้าเป็นทางออก
        self.connections = {}      # Dictionary เก็บเส้นเชื่อม: {neighbor_id: distance}
    
    # เพิ่ม repr เพื่อให้ print object แล้วแสดงผลสวยงาม
    def __repr__(self):
        return f"Node(id={self.id}, pos=({self.x},{self.y}), exit={self.is_exit})"

class Graph:
    def __init__(self):
        self.nodes = {}            # Dictionary เก็บโหนดทั้งหมด: {node_id: Node_object}
        self.node_count = 0

    def add_node(self, x, y, is_exit=False):
        new_id = self.node_count
        new_node = Node(new_id, x, y, is_exit)
        self.nodes[new_id] = new_node
        self.node_count += 1
        print(f"พบโหนดใหม่: {new_node}")
        return new_id

    def add_edge(self, from_node_id, to_node_id, distance):
        # สร้างเส้นเชื่อมไป-กลับ (Undirected Graph)
        if from_node_id in self.nodes and to_node_id in self.nodes:
            self.nodes[from_node_id].connections[to_node_id] = distance
            self.nodes[to_node_id].connections[from_node_id] = distance
            print(f"สร้างเส้นทางระหว่างโหนด {from_node_id} <-> {to_node_id} (ระยะทาง: {distance})")

# ============================================================
# == ส่วนที่ 2: อัลกอริทึมค้นหาเส้นทาง A* (ส่วนที่เพิ่มเข้ามา) ==
# ============================================================
def heuristic(node1, node2):
    # คำนวณระยะทางแบบยุคลิด (Euclidean distance) เพื่อใช้เป็นค่า Heuristic
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def a_star_search(graph, start_node_id, goal_node_id):
    """
    ค้นหาเส้นทางจาก start ไปยัง goal โดยใช้อัลกอริทึม A*
    """
    start_node = graph.nodes[start_node_id]
    goal_node = graph.nodes[goal_node_id]

    # Priority queue สำหรับเก็บ (f_score, node_id)
    open_set = [(0, start_node_id)] 
    
    # Dictionary สำหรับติดตามเส้นทางย้อนกลับ
    came_from = {}
    
    # g_score คือระยะทางจริงจากจุดเริ่มต้นมายังโหนดปัจจุบัน
    g_score = {node_id: float('inf') for node_id in graph.nodes}
    g_score[start_node_id] = 0
    
    # f_score คือ g_score + heuristic (ค่าประมาณระยะทางจากโหนดปัจจุบันไปยังเป้าหมาย)
    f_score = {node_id: float('inf') for node_id in graph.nodes}
    f_score[start_node_id] = heuristic(start_node, goal_node)

    while open_set:
        # ดึงโหนดที่มี f_score ต่ำที่สุดออกจาก queue
        _, current_id = heapq.heappop(open_set)

        # ถ้าถึงเป้าหมายแล้ว ให้สร้างเส้นทางและส่งคืน
        if current_id == goal_node_id:
            path = []
            total_distance = g_score[current_id]
            while current_id in came_from:
                path.append(current_id)
                current_id = came_from[current_id]
            path.append(start_node_id)
            return path[::-1], total_distance # คืนค่าเส้นทาง (เรียงจาก start -> goal) และระยะทางรวม

        # สำรวจโหนดเพื่อนบ้าน
        current_node = graph.nodes[current_id]
        for neighbor_id, distance in current_node.connections.items():
            # คำนวณ g_score ของเพื่อนบ้านผ่านโหนดปัจจุบัน
            tentative_g_score = g_score[current_id] + distance
            
            # ถ้าเส้นทางนี้ดีกว่าเส้นทางเดิมที่เคยพบ
            if tentative_g_score < g_score[neighbor_id]:
                came_from[neighbor_id] = current_id
                g_score[neighbor_id] = tentative_g_score
                f_score[neighbor_id] = tentative_g_score + heuristic(graph.nodes[neighbor_id], goal_node)
                
                # เพิ่มเพื่อนบ้านลงใน open_set เพื่อสำรวจต่อไป
                heapq.heappush(open_set, (f_score[neighbor_id], neighbor_id))

    return None, 0 # หากไม่พบเส้นทาง

# =================================================
# == ส่วนที่ 3: จำลองสถานการณ์ (ส่วนที่เพิ่มเข้ามา) ==
# =================================================
def simulate():
    print("--- 1. เริ่มการจำลอง: สร้างแผนที่ (Graph) ---")
    maze_map = Graph()

    # --- ขั้นตอนการเจอโหนดและสร้างแผนที่ ---
    # สมมติว่าหุ่นยนต์เคลื่อนที่ไปเจอโหนดต่างๆ และวัดระยะทาง
    start_id = maze_map.add_node(x=0, y=0)                 # โหนด 0 (จุดเริ่มต้น)
    node1_id = maze_map.add_node(x=10, y=0)                # โหนด 1
    node2_id = maze_map.add_node(x=10, y=10)               # โหนด 2
    node3_id = maze_map.add_node(x=0, y=10)                # โหนด 3
    exit_id  = maze_map.add_node(x=20, y=10, is_exit=True) # โหนด 4 (ทางออก)
    node5_id = maze_map.add_node(x=0, y=20)                # โหนด 5 (ทางตัน)

    print("\n--- 2. สร้างเส้นทางเชื่อมระหว่างโหนด ---")
    maze_map.add_edge(start_id, node1_id, 10)
    maze_map.add_edge(start_id, node3_id, 10)
    maze_map.add_edge(node1_id, node2_id, 10)
    maze_map.add_edge(node3_id, node2_id, 10)
    maze_map.add_edge(node2_id, exit_id, 10) # เส้นทางไปทางออก
    maze_map.add_edge(node3_id, node5_id, 10) # เส้นทางไปทางตัน

    # --- ขั้นตอนการค้นหาทางออก ---
    print(f"\n--- 3. ค้นหาเส้นทางจากโหนด {start_id} ไปยังทางออก (โหนด {exit_id}) ---")
    path, distance = a_star_search(maze_map, start_id, exit_id)

    if path:
        print(f"✅ พบเส้นทางแล้ว! ระยะทางรวม: {distance:.2f}")
        print(f"เส้นทางที่ต้องเดิน: {' -> '.join(map(str, path))}")

        # --- ขั้นตอนการย้อนกลับ ---
        print("\n--- 4. การเดินทางย้อนกลับ (Backtracking) ---")
        backtrack_path = path[::-1] # แค่ทำการกลับลำดับของ list
        print(f"หากต้องการย้อนกลับจากทางออก: {' -> '.join(map(str, backtrack_path))}")
    else:
        print("❌ ไม่พบเส้นทางไปยังทางออก")

# # เริ่มการทำงาน
# if __name__ == "__main__":
#     simulate()