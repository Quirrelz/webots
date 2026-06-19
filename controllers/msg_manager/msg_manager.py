from controller import Robot
import json
from pathlib import Path


ROBOT_ID = "msg_manager"
CONFIG_PATH = Path(__file__).resolve().parents[1] / "task_config.json"


def load_config():
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


config = load_config()
task_order = config["task_order"]
tasks = config["tasks"]
start_arm_id = config.get("start_arm_id", "start_arm")

cars = tuple(tasks[name]["car_id"] for name in task_order)
target_arms = tuple(tasks[name]["target_arm_id"] for name in task_order)

robot = Robot()
timestep = int(robot.getBasicTimeStep())

emitter = robot.getDevice("msg_manage_emitter")
receiver = robot.getDevice("msg_manage_receiver")
receiver.enable(timestep)

car_ready = {name: False for name in cars}
target_arm_ready = {name: False for name in target_arms}
task_state = {name: "pending_initial" for name in task_order}
target_go_sent_time = {}

last_ping_time = -1.0
ping_interval = float(config.get("startup_ping_interval", 0.5))
started = False
workflow_complete = False


def send_packet(target, command, data=None):
    packet = {
        "from": ROBOT_ID,
        "to": target,
        "command": command,
        "data": data or {},
        "time": robot.getTime(),
    }
    emitter.send(json.dumps(packet).encode("utf-8"))
    print("SEND:", packet)


def read_packets():
    packets = []

    while receiver.getQueueLength() > 0:
        raw = receiver.getString()

        try:
            packet = json.loads(raw)
        except json.JSONDecodeError:
            print("invalid packet:", raw)
            receiver.nextPacket()
            continue

        receiver.nextPacket()

        if packet.get("to") not in (ROBOT_ID, "broadcast"):
            continue

        if packet.get("from") == ROBOT_ID:
            continue

        packets.append(packet)

    return packets


def pose_to_go_to_data(pose):
    return {
        "target_x": float(pose["x"]),
        "target_y": float(pose["y"]),
        "target_heading": float(pose["heading"]),
    }


def start_initial_straight(task_name):
    task = tasks[task_name]
    send_packet(
        task["car_id"],
        "move_straight",
        {
            "distance": float(task["initial_straight_distance"]),
        },
    )
    task_state[task_name] = "wait_initial_done"
    print("task", task_name, "-> wait_initial_done")


def send_start_arm_pick(task_name):
    task = tasks[task_name]
    send_packet(
        start_arm_id,
        "pick_place_color",
        {
            "color": task["color"],
            "task": task_name,
        },
    )
    task_state[task_name] = "wait_start_arm_done"
    print("task", task_name, "-> wait_start_arm_done")


def send_car_to_target(task_name):
    task = tasks[task_name]
    send_packet(
        task["car_id"],
        "go_to",
        pose_to_go_to_data(task["target_pose"]),
    )
    target_go_sent_time[task_name] = robot.getTime()
    task_state[task_name] = "wait_target_go_done"
    print("task", task_name, "-> wait_target_go_done")


def send_target_approach(task_name):
    task = tasks[task_name]
    send_packet(
        task["car_id"],
        "move_straight",
        {
            "distance": float(task["target_approach_distance"]),
        },
    )
    task_state[task_name] = "wait_approach_done"
    print("task", task_name, "-> wait_approach_done")


def send_target_arm_pick(task_name):
    task = tasks[task_name]
    send_packet(
        task["target_arm_id"],
        "pick_place_color",
        {
            "color": task["color"],
            "task": task_name,
            "return_car_id": task["car_id"],
            "return_pose": task["return_pose"],
            "pre_return_straight_distance": float(task.get("pre_return_straight_distance", 0.0)),
            "vision_settle_time": float(task.get("target_vision_settle_time", 0.3)),
            "no_detection_miss_count": int(task.get("target_no_detection_miss_count", 80)),
        },
    )
    task_state[task_name] = "wait_target_arm_done"
    print("task", task_name, "-> wait_target_arm_done")


def handle_ready_packets(packets):
    for packet in packets:
        source = packet.get("from")
        command = packet.get("command")

        if source in car_ready and command == "car_ready":
            car_ready[source] = True
            send_packet(source, "init_ok")

        elif source in target_arm_ready and command == "target_arm_ready":
            target_arm_ready[source] = True
            send_packet(source, "init_ok")


def all_ready():
    return all(car_ready.values()) and all(target_arm_ready.values())


def task_for_car(car_id):
    for task_name in task_order:
        if tasks[task_name]["car_id"] == car_id:
            return task_name
    return None


def task_for_target_arm(arm_id):
    for task_name in task_order:
        if tasks[task_name]["target_arm_id"] == arm_id:
            return task_name
    return None


def process_task_packets(packets):
    for packet in packets:
        source = packet.get("from")
        command = packet.get("command")

        if command == "move_done":
            task_name = task_for_car(source)

            if task_name is None:
                continue

            state = task_state[task_name]

            if state == "wait_initial_done":
                send_start_arm_pick(task_name)

            elif state == "wait_target_go_done":
                send_target_approach(task_name)

            elif state == "wait_approach_done":
                send_target_arm_pick(task_name)

            elif state == "wait_return_done":
                task_state[task_name] = "complete"
                print("task", task_name, "return complete")

        elif command == "arm_done" and source == start_arm_id:
            color = packet.get("data", {}).get("color")

            for task_name in task_order:
                if tasks[task_name]["color"] == color and task_state[task_name] == "wait_start_arm_done":
                    send_car_to_target(task_name)
                    break

        elif command == "target_arm_done":
            task_name = task_for_target_arm(source)

            if task_name is not None:
                task_state[task_name] = "wait_return_done"
                print("task", task_name, "target arm done; return command is sent by", source)

        elif command in ("arm_failed", "target_arm_failed"):
            print("failure packet:", packet)


def start_delayed_tasks(now):
    for index, task_name in enumerate(task_order[:-1]):
        next_task_name = task_order[index + 1]

        if task_state[next_task_name] != "pending_initial":
            continue

        if task_name not in target_go_sent_time:
            continue

        delay = float(tasks[task_name].get("next_car_delay", 2.0))

        if now - target_go_sent_time[task_name] >= delay:
            start_initial_straight(next_task_name)


print("msg_manager started")

while robot.step(timestep) != -1:
    now = robot.getTime()
    packets = read_packets()

    if not started:
        if now - last_ping_time >= ping_interval:
            last_ping_time = now

            for car_name in cars:
                if not car_ready[car_name]:
                    send_packet(car_name, "manager_ping")

            for arm_name in target_arms:
                if not target_arm_ready[arm_name]:
                    send_packet(arm_name, "manager_ping")

        handle_ready_packets(packets)

        if all_ready():
            print("all cars and target arms ready")
            started = True
            start_initial_straight(task_order[0])

        continue

    process_task_packets(packets)
    start_delayed_tasks(now)

    if not workflow_complete and all(task_state[name] == "complete" for name in task_order):
        print("all tasks complete")
        workflow_complete = True
