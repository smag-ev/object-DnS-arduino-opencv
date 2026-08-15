import csv
import datetime
import heapq
import os
import threading
import time

import cv2
import serial

try:
    import winsound
except ImportError:
    winsound = None

# Configuration
CAMERA_URL = "URL HERE"
SERIAL_PORT = "COM9"
BAUD_RATE = 115200

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 360

TRACKER_TYPE = "CSRT"
INVERT_SERVO = False
SMOOTHING = 0.5
SERVO_DEADZONE = 2

SERVO_MIN = 30
SERVO_MAX = 150
REST_ANGLE = 90
EDGE_MARGIN = 0.10

LOCK_FRAMES = 15
TRACKING_COOLDOWN = 2.0

SIZE_MIN_FACTOR = 0.7
SIZE_MAX_FACTOR = 1.5
SIZE_REJECT_FACTOR = 4.0

TRACKING_SOUND_REPEATS = 2
LOCK_SOUND_REPEATS = 2

GRID_COLUMNS = 40
GRID_ROWS = 23
PLANNER_INTERVAL = 1.0

LOG_FILE = "tracking_log.csv"
TRACKING_SOUND = "tracking.wav"
LOCK_SOUND = "lock.wav"

FONT = cv2.FONT_HERSHEY_SIMPLEX

log_lock = threading.Lock()
log_file = None
log_writer = None


def initialize_logger():
    global log_file, log_writer
    log_file = open(LOG_FILE, "w", newline="", encoding="utf-8")
    log_writer = csv.writer(log_file)
    log_writer.writerow(["timestamp", "event", "detail"])
    log_file.flush()


def log_event(event, detail=""):
    if log_writer is None:
        return

    with log_lock:
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        log_writer.writerow([timestamp, event, detail])
        log_file.flush()


def close_logger():
    global log_file
    if log_file is not None:
        log_file.flush()
        log_file.close()
        log_file = None


def play_sound(filename, repeats=1):
    if winsound is None or not os.path.exists(filename):
        return

    def worker():
        for _ in range(repeats):
            winsound.PlaySound(filename, winsound.SND_FILENAME)

    threading.Thread(target=worker, daemon=True).start()


def create_tracker(kind=TRACKER_TYPE):
    kind = kind.upper()

    def create(name, params=None):
        for namespace in (cv2, getattr(cv2, "legacy", None)):
            if namespace is None:
                continue

            factory = getattr(namespace, f"Tracker{name}_create", None)
            if factory is None:
                continue

            try:
                return factory(params) if params is not None else factory()
            except Exception:
                try:
                    return factory()
                except Exception:
                    continue

        return None

    if kind == "CSRT":
        params_type = getattr(cv2, "TrackerCSRT_Params", None)
        if params_type is None and hasattr(cv2, "legacy"):
            params_type = getattr(cv2.legacy, "TrackerCSRT_Params", None)

        if params_type is not None:
            try:
                params = params_type()
                if hasattr(params, "use_segmentation"):
                    params.use_segmentation = False

                tracker = create("CSRT", params)
                if tracker is not None:
                    return tracker
            except Exception:
                pass

    tracker = create(kind) or create("CSRT") or create("KCF")

    if tracker is None:
        raise RuntimeError(
            "No compatible OpenCV tracker found. "
            "Install opencv-contrib-python."
        )

    return tracker


CELL_WIDTH = DISPLAY_WIDTH / GRID_COLUMNS
CELL_HEIGHT = DISPLAY_HEIGHT / GRID_ROWS

MOVES = [
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, 1.41421356),
    (-1, 1, 1.41421356),
    (1, -1, 1.41421356),
    (1, 1, 1.41421356),
]


def cell_of(px, py):
    column = min(max(int(px / CELL_WIDTH), 0), GRID_COLUMNS - 1)
    row = min(max(int(py / CELL_HEIGHT), 0), GRID_ROWS - 1)
    return column, row


def astar(start, goal):
    def heuristic(a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    queue = [(heuristic(start, goal), 0.0, start)]
    came_from = {}
    scores = {start: 0.0}
    closed = set()
    expanded = 0

    while queue:
        _, cost, current = heapq.heappop(queue)

        if current in closed:
            continue

        closed.add(current)
        expanded += 1

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)

            path.reverse()
            return path, cost, expanded

        for dx, dy, step_cost in MOVES:
            nx = current[0] + dx
            ny = current[1] + dy

            if not (0 <= nx < GRID_COLUMNS and 0 <= ny < GRID_ROWS):
                continue

            neighbor = nx, ny
            new_cost = cost + step_cost

            if new_cost < scores.get(neighbor, float("inf")):
                scores[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + heuristic(neighbor, goal)
                heapq.heappush(queue, (priority, new_cost, neighbor))

    return None, float("inf"), expanded


plan_state = {"target": None, "servo": REST_ANGLE}
plan_lock = threading.Lock()

display_frame = None
camera_lock = threading.Lock()
camera_running = True


def planner_thread():
    while camera_running:
        time.sleep(PLANNER_INTERVAL)

        with plan_lock:
            target = plan_state["target"]
            servo = plan_state["servo"]

        if target is None:
            continue

        start = cell_of(
            (servo / 180.0) * DISPLAY_WIDTH,
            DISPLAY_HEIGHT / 2,
        )
        goal = cell_of(*target)

        path, cost, expanded = astar(start, goal)

        if path:
            log_event(
                "ASTAR",
                f"goal={goal} cost={cost:.2f} "
                f"nodes={expanded} waypoints={len(path)}",
            )


def camera_thread(camera_url):
    global display_frame

    capture = cv2.VideoCapture(camera_url)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not capture.isOpened():
        log_event("CAMERA_ERROR", str(camera_url))
        return

    while camera_running:
        ok, frame = capture.read()

        if not ok:
            time.sleep(0.05)
            continue

        frame = cv2.flip(frame, 1)
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        frame = cv2.resize(
            frame,
            (DISPLAY_WIDTH, DISPLAY_HEIGHT),
            interpolation=cv2.INTER_LINEAR,
        )

        with camera_lock:
            display_frame = frame

    capture.release()


def send_command(connection, command):
    connection.write(f"{command}\n".encode("ascii"))


def open_serial():
    try:
        connection = serial.Serial(
            SERIAL_PORT,
            BAUD_RATE,
            timeout=1,
        )
        time.sleep(2)
        return connection
    except serial.SerialException as exc:
        raise RuntimeError(
            f"Could not open serial port {SERIAL_PORT}: {exc}"
        ) from exc


def main():
    global camera_running

    initialize_logger()
    serial_connection = open_serial()

    threading.Thread(
        target=camera_thread,
        args=(CAMERA_URL,),
        daemon=True,
    ).start()

    threading.Thread(
        target=planner_thread,
        daemon=True,
    ).start()

    time.sleep(1)

    tracker = None
    target_acquired = False

    initial_width = 0
    initial_height = 0
    initial_area = 1

    previous_center = (0, 0)

    servo_filtered = float(REST_ANGLE)
    servo_angle = REST_ANGLE
    last_sent = -1

    stable_count = 0
    locked = False
    lock_sound_played = False
    last_tracking_sound = 0.0

    previous_ok = False
    previous_locked = False
    lock_count = 0

    cv2.namedWindow("OBJECT TRACKING", cv2.WINDOW_NORMAL)

    def acquire_target():
        nonlocal tracker
        nonlocal target_acquired
        nonlocal initial_width, initial_height, initial_area
        nonlocal previous_center, stable_count, locked
        nonlocal lock_sound_played

        with camera_lock:
            frame = None if display_frame is None else display_frame.copy()

        if frame is None:
            return False

        box = cv2.selectROI(
            "OBJECT TRACKING",
            frame,
            showCrosshair=True,
            fromCenter=False,
        )

        if box == (0, 0, 0, 0):
            return False

        tracker = create_tracker()
        tracker.init(frame, box)

        initial_width = int(box[2])
        initial_height = int(box[3])
        initial_area = max(1, initial_width * initial_height)

        previous_center = (
            int(box[0] + box[2] / 2),
            int(box[1] + box[3] / 2),
        )

        target_acquired = True
        stable_count = 0
        locked = False
        lock_sound_played = False

        return True

    def reset_to_rest():
        nonlocal tracker, target_acquired
        nonlocal servo_angle, servo_filtered, last_sent
        nonlocal stable_count, locked, lock_sound_played

        send_command(serial_connection, f"R{REST_ANGLE}")

        tracker = None
        target_acquired = False

        servo_angle = REST_ANGLE
        servo_filtered = float(REST_ANGLE)
        last_sent = -1

        stable_count = 0
        locked = False
        lock_sound_played = False

        with plan_lock:
            plan_state["target"] = None

    send_command(serial_connection, f"R{REST_ANGLE}")
    log_event(
        "SESSION_START",
        f"tracker={TRACKER_TYPE} servo_range={SERVO_MIN}-{SERVO_MAX}",
    )

    print("Select the object to track and press ENTER or SPACE.")
    print("Controls: N = new target, R = reset, I = invert, Q = quit")

    acquire_target()

    try:
        while True:
            with camera_lock:
                if display_frame is None:
                    time.sleep(0.005)
                    continue

                frame = display_frame.copy()

            height, width = frame.shape[:2]
            ok = False

            if target_acquired and tracker is not None:
                ok, box = tracker.update(frame)

            if ok:
                rx, ry, rw, rh = [int(value) for value in box]
                area_ratio = (rw * rh) / float(initial_area)

                if area_ratio > SIZE_REJECT_FACTOR:
                    ok = False
                else:
                    cx = rx + rw // 2
                    cy = ry + rh // 2

                    box_width = int(
                        min(
                            max(rw, initial_width * SIZE_MIN_FACTOR),
                            initial_width * SIZE_MAX_FACTOR,
                        )
                    )
                    box_height = int(
                        min(
                            max(rh, initial_height * SIZE_MIN_FACTOR),
                            initial_height * SIZE_MAX_FACTOR,
                        )
                    )

                    previous_x, previous_y = previous_center
                    dx = cx - previous_x
                    dy = cy - previous_y
                    distance = (dx * dx + dy * dy) ** 0.5
                    max_step = max(45, max(box_width, box_height))

                    if distance > max_step:
                        scale = max_step / distance
                        cx = int(previous_x + dx * scale)
                        cy = int(previous_y + dy * scale)

                    previous_center = cx, cy

                    x1 = cx - box_width // 2
                    y1 = cy - box_height // 2
                    x2 = x1 + box_width
                    y2 = y1 + box_height

            if ok:
                stable_count = min(stable_count + 1, LOCK_FRAMES)
                locked = stable_count >= LOCK_FRAMES

                box_color = (
                    (0, 0, 255)
                    if locked
                    else (0, 165, 255)
                    if stable_count > LOCK_FRAMES // 2
                    else (0, 255, 0)
                )

                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    box_color,
                    3,
                )
                cv2.circle(frame, (cx, cy), 5, box_color, -1)

                left = EDGE_MARGIN * width
                right = (1.0 - EDGE_MARGIN) * width
                constrained_x = min(max(cx, left), right)

                fraction = (constrained_x - left) / max(
                    1.0,
                    right - left,
                )

                if INVERT_SERVO:
                    fraction = 1.0 - fraction

                target_angle = int(
                    SERVO_MIN
                    + fraction * (SERVO_MAX - SERVO_MIN)
                )

                target_angle = max(
                    SERVO_MIN,
                    min(SERVO_MAX, target_angle),
                )

                servo_filtered += SMOOTHING * (
                    target_angle - servo_filtered
                )

                servo_angle = max(
                    SERVO_MIN,
                    min(SERVO_MAX, int(round(servo_filtered))),
                )

                if abs(servo_angle - last_sent) >= SERVO_DEADZONE:
                    send_command(
                        serial_connection,
                        f"S{servo_angle}",
                    )
                    last_sent = servo_angle

                with plan_lock:
                    plan_state["target"] = (cx, cy)
                    plan_state["servo"] = servo_angle

                now = time.time()

                if (
                    not locked
                    and now - last_tracking_sound > TRACKING_COOLDOWN
                ):
                    play_sound(
                        TRACKING_SOUND,
                        TRACKING_SOUND_REPEATS,
                    )
                    last_tracking_sound = now

                if locked and not lock_sound_played:
                    play_sound(
                        LOCK_SOUND,
                        LOCK_SOUND_REPEATS,
                    )
                    lock_sound_played = True

            else:
                stable_count = max(stable_count - 1, 0)

                if stable_count == 0:
                    locked = False
                    lock_sound_played = False

                    if last_sent != -1:
                        send_command(serial_connection, "STOP")
                        last_sent = -1

                    with plan_lock:
                        plan_state["target"] = None

            if ok and not previous_ok:
                log_event("TARGET_ACQUIRED")

            if previous_ok and not ok:
                log_event("TARGET_LOST")

            if locked and not previous_locked:
                lock_count += 1
                log_event("LOCKED", f"servo={servo_angle}")

            previous_ok = ok
            previous_locked = locked

            status = (
                "LOCKED"
                if locked
                else "TRACKING"
                if ok
                else "NO TARGET"
            )

            cv2.putText(
                frame,
                f"STATUS: {status}",
                (20, 35),
                FONT,
                0.8,
                (255, 255, 255),
                2,
            )

            cv2.imshow("OBJECT TRACKING", frame)

            key = cv2.waitKey(1) & 0xFF

            # FIRE COMMAND PLACEHOLDER
            # Physical actuation intentionally omitted from this repository copy.

            if key == ord("q"):
                break
            elif key == ord('s'):
                ser.write(b"FIRE\n")
                play_sound("fire.wav", times=FIRE_TIMES)
            elif key == ord("n"):
                reset_to_rest()
                acquire_target()
            elif key == ord("r"):
                reset_to_rest()
            elif key == ord("i"):
                INVERT_SERVO = not INVERT_SERVO
                log_event("INVERT", str(INVERT_SERVO))

    finally:
        camera_running = False

        try:
            send_command(serial_connection, "STOP")
            serial_connection.close()
        finally:
            log_event("SESSION_END", f"locks={lock_count}")
            close_logger()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
