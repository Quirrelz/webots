from controller import Supervisor
import json
import math
import heapq
from pathlib import Path

MAP_PATH = Path(__file__).resolve().parents[1] / "maps" / "obstacle_grid.json"

ROSBOT_LENGTH = 0.332
ROSBOT_WIDTH = 0.284

# 如果小车允许原地转向，建议用半对角线作为保守碰撞半径
ROBOT_PLANNING_RADIUS = math.sqrt((ROSBOT_LENGTH / 2) ** 2 + (ROSBOT_WIDTH / 2) ** 2)

WAYPOINT_TOLERANCE = 0.12
FINAL_TARGET_TOLERANCE = 0.15
FINAL_HEADING_TOLERANCE = 0.08

MAX_LINEAR_SPEED = 0.6
MAX_ANGULAR_SPEED = 1.2

MAX_LINEAR_ACCEL = 0.25
MAX_ANGULAR_ACCEL = 1.0

TURN_IN_PLACE_ANGULAR = 0.45
TURN_IN_PLACE_COMMAND = 0.8

SLOW_TURN_ANGULAR = 0.18
SLOW_TURN_LINEAR_SCALE = 0.55
def approach(current, target, max_delta):
    if target > current:
        return min(target, current + max_delta)
    return max(target, current - max_delta)
class BaseCarController:
    def __init__(
        self,
        robot_id,
        emitter_name,
        receiver_name,
        motor_names=None,
        manager_id="msg_manager",
    ):
        self.robot_id = robot_id
        self.manager_id = manager_id

        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.self_node = self.robot.getSelf()


        if motor_names is None:
            motor_names = {
                "front_left": "fl_wheel_joint",
                "front_right": "fr_wheel_joint",
                "rear_left": "rl_wheel_joint",
                "rear_right": "rr_wheel_joint",
            }

        self.fl_motor = self.robot.getDevice(motor_names["front_left"])
        self.fr_motor = self.robot.getDevice(motor_names["front_right"])
        self.rl_motor = self.robot.getDevice(motor_names["rear_left"])
        self.rr_motor = self.robot.getDevice(motor_names["rear_right"])

        self.motors = [
            self.fl_motor,
            self.fr_motor,
            self.rl_motor,
            self.rr_motor,
        ]

        for motor in self.motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        self.wheel_radius = 0.05
        self.base_lx = 0.085
        self.base_ly = 0.135
        self.base_k = self.base_lx + self.base_ly
        self.max_wheel_speed = 20.0

        self.emitter = self.robot.getDevice(emitter_name)

        self.receiver = self.robot.getDevice(receiver_name)
        self.receiver.enable(self.timestep)

        self.motion_mode = "idle"
        self.target_x = None
        self.target_y = None
        self.target_heading = None
        self.pending_target_after_straight = None
        self.init_ok = False
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.drive_mode = "idle"
        self.map_data = self.load_grid_map()
        self.path_cells = []
        self.path_world = []
        self.path_index = 0
        self.pending_move_done_after_heading = False
        self.command_queue = []
        self.active_notify_done = True

    def send_packet(self, target, command, data=None):
        packet = {
            "from": self.robot_id,
            "to": target,
            "command": command,
            "data": data or {},
            "time": self.robot.getTime(),
        }

        self.emitter.send(json.dumps(packet).encode("utf-8"))

    def read_packets(self):
        packets = []

        while self.receiver.getQueueLength() > 0:
            raw = self.receiver.getString()

            try:
                packet = json.loads(raw)
            except json.JSONDecodeError:
                print(self.robot_id, "received invalid packet:", raw)
                self.receiver.nextPacket()
                continue

            if packet.get("to") not in (self.robot_id, "broadcast"):
                self.receiver.nextPacket()
                continue

            if packet.get("from") == self.robot_id:
                self.receiver.nextPacket()
                continue

            packets.append(packet)

            self.receiver.nextPacket()

        return packets

    def handle_init_message(self, packet):
        if packet.get("to") != self.robot_id:
            return

        source = packet.get("from")
        command = packet.get("command")

        print(self.robot_id, "received:", packet)

        if source == self.manager_id and command == "manager_ping":
            self.send_packet(self.manager_id, "car_ready")
            print(self.robot_id, "->", self.manager_id, "car_ready")

        elif source == self.manager_id and command == "init_ok":
            self.init_ok = True
            print(self.robot_id, "communication initialized successfully")

    def update_communication(self):
        for packet in self.read_packets():
            self.handle_init_message(packet)
            self.handle_motion_command(packet)

    def set_chassis_speed(self, vx, vy, wz):
        forward = vx / self.wheel_radius
        turn = wz * self.base_k / self.wheel_radius

        fl = forward - turn
        fr = forward + turn
        rl = forward - turn
        rr = forward + turn

        speeds = [fl, fr, rl, rr]
        max_abs = max(abs(v) for v in speeds)

        if max_abs > self.max_wheel_speed:
            scale = self.max_wheel_speed / max_abs
            speeds = [v * scale for v in speeds]

        self.fl_motor.setVelocity(speeds[0])
        self.fr_motor.setVelocity(speeds[1])
        self.rl_motor.setVelocity(speeds[2])
        self.rr_motor.setVelocity(speeds[3])

    def stop(self):
        self.set_chassis_speed(0.0, 0.0, 0.0)
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.drive_mode = "idle"

    def get_position(self):
        pos = self.self_node.getPosition()
        return pos[0], pos[1]
    
    def get_heading(self):
        orientation = self.self_node.getOrientation()

        # 车体本地 X 轴在世界坐标里的方向
        forward_x = orientation[0]
        forward_y = orientation[3]

        return self.normalize_angle(math.atan2(forward_y, forward_x))

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def load_grid_map(self):
        with open(MAP_PATH, "r", encoding="utf-8") as f:
            return json.load(f)

    def world_to_grid(self, x, y):
        grid = self.map_data["grid"]
        origin_x, origin_y = grid["origin"]
        resolution = grid["resolution"]

        row = int(math.floor((y - origin_y) / resolution))
        col = int(math.floor((x - origin_x) / resolution))
        return row, col

    def grid_to_world(self, row, col):
        grid = self.map_data["grid"]
        origin_x, origin_y = grid["origin"]
        resolution = grid["resolution"]

        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution
        return x, y

    def build_planning_occupancy(self):
        grid = self.map_data["grid"]
        width = int(grid["width"])
        height = int(grid["height"])
        resolution = float(grid["resolution"])

        blocked = set(tuple(cell) for cell in self.map_data["occupied_cells"])

        # 路径规划阶段按小车大小膨胀障碍，不改地图本体
        inflation_cells = int(math.ceil(ROBOT_PLANNING_RADIUS / resolution))
        planning_blocked = set(blocked)

        for row, col in blocked:
            for dr in range(-inflation_cells, inflation_cells + 1):
                for dc in range(-inflation_cells, inflation_cells + 1):
                    nr = row + dr
                    nc = col + dc

                    if nr < 0 or nr >= height or nc < 0 or nc >= width:
                        continue

                    distance = math.sqrt((dr * resolution) ** 2 + (dc * resolution) ** 2)
                    if distance <= ROBOT_PLANNING_RADIUS:
                        planning_blocked.add((nr, nc))

        return planning_blocked

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def astar(self, start, goal):
        grid = self.map_data["grid"]
        width = int(grid["width"])
        height = int(grid["height"])
        blocked = self.build_planning_occupancy()

        if start in blocked:
            print(self.robot_id, "A* start cell is blocked:", start)
            return []

        if goal in blocked:
            print(self.robot_id, "A* goal cell is blocked:", goal)
            return []

        neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (1, 1, math.sqrt(2)),
        ]

        open_heap = []
        heapq.heappush(open_heap, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}
        closed = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in closed:
                continue

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            closed.add(current)
            row, col = current

            for dr, dc, move_cost in neighbors:
                nr = row + dr
                nc = col + dc
                neighbor = (nr, nc)

                if nr < 0 or nr >= height or nc < 0 or nc >= width:
                    continue

                if neighbor in blocked:
                    continue

                # 防止斜向穿过墙角
                if dr != 0 and dc != 0:
                    if (row + dr, col) in blocked or (row, col + dc) in blocked:
                        continue

                tentative_g = g_score[current] + move_cost

                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f_score, neighbor))

        print(self.robot_id, "A* failed:", start, "->", goal)
        return []

    def simplify_path(self, path):
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        prev_dr = None
        prev_dc = None

        for i in range(1, len(path)):
            dr = path[i][0] - path[i - 1][0]
            dc = path[i][1] - path[i - 1][1]

            if prev_dr is not None and (dr, dc) != (prev_dr, prev_dc):
                simplified.append(path[i - 1])

            prev_dr = dr
            prev_dc = dc

        simplified.append(path[-1])
        return simplified

    def plan_path_to_target(self, target_x, target_y):
        x, y = self.get_position()

        start = self.world_to_grid(x, y)
        goal = self.world_to_grid(target_x, target_y)

        path = self.astar(start, goal)
        path = self.simplify_path(path)

        self.path_cells = path
        self.path_world = [self.grid_to_world(row, col) for row, col in path]
        self.path_index = 0

        print(self.robot_id, "planned path cells:", len(self.path_cells))
        print(self.robot_id, "planned path:", self.path_world)

        return len(self.path_world) > 0

    def update_path_following(self):
        if not self.path_world:
            self.stop()
            return True

        if self.path_index >= len(self.path_world):
            self.stop()
            return True

        waypoint_x, waypoint_y = self.path_world[self.path_index]

        is_final = self.path_index == len(self.path_world) - 1
        tolerance = FINAL_TARGET_TOLERANCE if is_final else WAYPOINT_TOLERANCE

        reached = self.go_to_target(
            waypoint_x,
            waypoint_y,
            max_linear_speed=0.45,
            position_gain=0.9,
            heading_gain=1.8,
            distance_tolerance=tolerance,
        )

        if reached:
            if is_final and self.target_heading is not None:
                self.motion_mode = "align_heading"
                self.pending_move_done_after_heading = True
                return False

            self.path_index += 1

            if self.path_index >= len(self.path_world):
                self.stop()
                print(self.robot_id, "path navigation done")
                return True

        return False

    def align_to_heading(
        self,
        target_heading,
        heading_gain=1.6,
        heading_tolerance=FINAL_HEADING_TOLERANCE,
    ):
        heading = self.get_heading()
        heading_error = self.normalize_angle(target_heading - heading)

        if abs(heading_error) < heading_tolerance:
            self.stop()
            print(self.robot_id, "final heading reached:", round(target_heading, 3))
            return True

        target_angular = heading_gain * heading_error
        target_angular = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, target_angular))

        if abs(target_angular) < 0.25:
            target_angular = math.copysign(0.25, target_angular)

        dt = self.timestep / 1000.0
        self.current_linear = approach(self.current_linear, 0.0, MAX_LINEAR_ACCEL * dt)
        self.current_angular = approach(
            self.current_angular,
            target_angular,
            MAX_ANGULAR_ACCEL * dt,
        )
        self.drive_mode = "align-heading"
        self.set_chassis_speed(0.0, 0.0, self.current_angular)
        return False
   
    def go_to_target(
        self,
        target_x,
        target_y,
        max_linear_speed=0.6,
        position_gain=0.8,
        heading_gain=1.5,
        distance_tolerance=0.15,
        turn_sign=1.0,
    ):
        x, y = self.get_position()

        dx = target_x - x
        dy = target_y - y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < distance_tolerance:
            self.stop()
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.drive_mode = "arrived"
            print(self.robot_id, "reached target:", target_x, target_y)
            return True

        heading = self.get_heading()
        target_angle = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_angle - heading)

        target_linear = min(max_linear_speed, position_gain * distance)
        target_angular = heading_gain * heading_error * turn_sign

        target_linear, target_angular, self.drive_mode = self.drive_command_from_target(
            target_linear,
            target_angular,
        )

        dt = self.timestep / 1000.0

        self.current_linear = approach(
            self.current_linear,
            target_linear,
            MAX_LINEAR_ACCEL * dt,
        )

        self.current_angular = approach(
            self.current_angular,
            target_angular,
            MAX_ANGULAR_ACCEL * dt,
        )

        self.set_chassis_speed(self.current_linear, 0.0, self.current_angular)

        return False

    def step(self):
        return self.robot.step(self.timestep)
    
    def start_move_straight(self, distance, speed=0.4):
        self.motion_mode = "move_straight"
        self.straight_start_x = None
        self.straight_start_y = None
        self.straight_distance = distance
        self.straight_speed = math.copysign(abs(speed), distance)

        print(self.robot_id, "start move straight:", distance)

    def update_move_straight(self):
        x, y = self.get_position()

        if not math.isfinite(x) or not math.isfinite(y):
            print(self.robot_id, "waiting for valid GPS:", x, y)
            self.stop()
            return False

        if self.straight_start_x is None or self.straight_start_y is None:
            self.straight_start_x = x
            self.straight_start_y = y
            print(self.robot_id, "straight start position:", x, y)
            return False

        dx = x - self.straight_start_x
        dy = y - self.straight_start_y
        moved = math.sqrt(dx * dx + dy * dy)

        print(self.robot_id, "moved:", round(moved, 3), "target:", self.straight_distance)

        if moved >= abs(self.straight_distance):
            self.stop()
            self.motion_mode = "idle"
            print(self.robot_id, "straight move done:", moved)
            return True

        self.set_chassis_speed(self.straight_speed, 0.0, 0.0)
        return False
    
    def handle_motion_command(self, packet):
        if packet.get("to") != self.robot_id:
            return

        command = packet.get("command")
        data = packet.get("data", {})

        if command in ("move_straight", "go_to") and self.motion_mode != "idle":
            self.command_queue.append(packet)
            print(self.robot_id, "queued command:", command, data)
            return

        if command == "move_straight":
            self.pending_target_after_straight = None
            self.active_notify_done = bool(data.get("notify_done", True))
            self.start_move_straight(float(data["distance"]))

        elif command == "go_to":
            self.active_notify_done = bool(data.get("notify_done", True))
            straight_first = float(data.get("straight_first", 0.0))
            self.target_x = float(data["target_x"])
            self.target_y = float(data["target_y"])
            self.target_heading = data.get("target_heading")

            if self.target_heading is not None:
                self.target_heading = float(self.target_heading)

            if straight_first > 0:
                self.pending_target_after_straight = (
                    self.target_x,
                    self.target_y,
                    self.target_heading,
                )
                self.start_move_straight(straight_first)
            else:
                if self.plan_path_to_target(self.target_x, self.target_y):
                    self.motion_mode = "path_follow"
                else:
                    self.stop()
                    self.motion_mode = "idle"
    
    def update_motion(self):
        if self.motion_mode == "move_straight":
            done = self.update_move_straight()

            if done:
                if self.pending_target_after_straight is not None:
                    (
                        self.target_x,
                        self.target_y,
                        self.target_heading,
                    ) = self.pending_target_after_straight
                    self.pending_target_after_straight = None
                    self.motion_mode = "go_to"
                else:
                    if self.active_notify_done:
                        self.send_packet("msg_manager", "move_done")

                    self.start_next_queued_command()

        elif self.motion_mode == "go_to":
            if self.plan_path_to_target(self.target_x, self.target_y):
                self.motion_mode = "path_follow"
            else:
                self.stop()
                self.motion_mode = "idle"

        elif self.motion_mode == "path_follow":
            reached = self.update_path_following()

            if reached:
                self.motion_mode = "idle"
                if self.active_notify_done:
                    self.send_packet("msg_manager", "move_done")

                self.start_next_queued_command()

        elif self.motion_mode == "align_heading":
            reached = self.align_to_heading(self.target_heading)

            if reached:
                self.motion_mode = "idle"

                if self.pending_move_done_after_heading:
                    self.pending_move_done_after_heading = False
                    if self.active_notify_done:
                        self.send_packet("msg_manager", "move_done")

                    self.start_next_queued_command()

    def start_next_queued_command(self):
        if self.motion_mode != "idle" or not self.command_queue:
            return

        next_packet = self.command_queue.pop(0)
        print(self.robot_id, "start queued command:", next_packet.get("command"), next_packet.get("data", {}))
        self.handle_motion_command(next_packet)
    
    def drive_command_from_target(self, target_linear, target_angular):
        target_linear = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, target_linear))
        target_angular = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, target_angular))

        drive_mode = "track"

        if abs(target_angular) >= TURN_IN_PLACE_ANGULAR:
            target_linear = 0.0
            target_angular = math.copysign(TURN_IN_PLACE_COMMAND, target_angular)
            drive_mode = "turn-align"

        elif abs(target_angular) >= SLOW_TURN_ANGULAR:
            target_linear *= SLOW_TURN_LINEAR_SCALE
            drive_mode = "slow-turn"

        return target_linear, target_angular, drive_mode
