"""Maze - YOUR CONTROLLER / TWOJ STEROWNIK.

Goal: drive from the start to the green goal area as fast as possible,
without touching the walls.

Devices of the e-puck robot:
  - 'left wheel motor', 'right wheel motor'   - wheels, max speed 6.28 rad/s
  - 'left wheel sensor', 'right wheel sensor' - wheel encoders [rad]
  - 'ps0' ... 'ps7'  - infrared distance sensors around the robot
        ps0, ps7 - front, ps1 - front-right, ps2 - right, ps5 - left, ps6 - front-left
        ps3, ps4 - back
        about 65-70 = nothing nearby, about 150 = wall 4 cm away, more = closer
  - 'gs0', 'gs1', 'gs2' - ground sensors (the green goal area is darker than the floor)

Maze geometry: cells are 12 cm x 12 cm, walls are 1 cm thick.
Wheel radius: 2.05 cm, distance between the wheels: 5.2 cm.

Fill in the parts marked TODO.
"""
import math
from controller import Robot

# parametry robota
MAX_SPEED = 6.28
WHEEL_RADIUS = 0.0205
WHEELS_DIST = 0.052
WALL_CLOSE = 100
SIDE_WALL = 110
FRONT_STOP = 150.0
KP_BOTH = 0.003
KP_ONE = 0.004
KP_ENC = 2
MAX_CORR = 0.35

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- Motors ------------------------------------------------------------------
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))   # velocity control mode
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# --- Wheel encoders ----------------------------------------------------------
left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(timestep)
right_encoder.enable(timestep)


# --- Distance sensors --------------------------------------------------------
distance_sensors = []
for i in range(8):
    sensor = robot.getDevice(f"ps{i}")
    sensor.enable(timestep)
    distance_sensors.append(sensor)

# --- Ground sensors ----------------------------------------------------------
ground_sensors = []
for name in ["gs0", "gs1", "gs2"]:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    ground_sensors.append(sensor)


def clamp(value, low=-MAX_SPEED, high=MAX_SPEED):
    return max(low, min(high, value))
    
def is_wall_in_front():
    return distance_sensors[0].getValue() > WALL_CLOSE or distance_sensors[7].getValue() > WALL_CLOSE
    
def is_wall_on_right():
    return (distance_sensors[2].getValue() > WALL_CLOSE)
    
def is_wall_on_left():
    return (distance_sensors[5].getValue() > WALL_CLOSE 
    or distance_sensors[6].getValue() > WALL_CLOSE)


class DriveCorrector:
    def __init__(self):
        self.reset()

    def reset(self):
        self.mode = None
        self.ref = None          # setpoint dla trybu 'left'/'right' lub (l,r) enkoderów dla 'enc'
        self.last_corr = 0.0
        self.walls_lost = False

    def __call__(self, base_speed):
        left = distance_sensors[5].getValue()
        right = distance_sensors[2].getValue()
        has_l = left > SIDE_WALL
        has_r = right > SIDE_WALL

        if has_l and has_r:
            mode = "both"
        elif has_l:
            mode = "left"
        elif has_r:
            mode = "right"
        else:
            mode = "enc"

        # mode sie zmienil, czyli ubyla lub przybyla sciana
        if self.mode is not None and mode != self.mode:
            prev_walls = {"both": 2, "left": 1, "right": 1, "enc": 0}
            if prev_walls[mode] < prev_walls[self.mode]:
                self.walls_lost = True
            # przy przelaczeniu: zapamietaj aktualny odczyt jako setpoint
            if mode == "left":
                self.ref = left
            elif mode == "right":
                self.ref = right
            elif mode == "enc":
                self.ref = (left_encoder.getValue(), right_encoder.getValue())
        elif self.mode is None:
            self.ref = {"left": left, "right": right,
                        "enc": (left_encoder.getValue(), right_encoder.getValue())}.get(mode)
        self.mode = mode

        if self.walls_lost:
            error = 0.0
        elif mode == "both":
            error = (right - left) * KP_BOTH
        elif mode == "left":
            error = (self.ref - left) * KP_ONE
        elif mode == "right":
            error = (right - self.ref) * KP_ONE
        else:
            dl = left_encoder.getValue() - self.ref[0]
            dr = right_encoder.getValue() - self.ref[1]
            error = (dl - dr) * KP_ENC

        corr = clamp(error, -MAX_CORR, MAX_CORR)
        # rate limit: max zmiana 0.05 rad/s na krok
        corr = clamp(corr, self.last_corr - 0.05, self.last_corr + 0.05)
        self.last_corr = corr
        return clamp(base_speed - corr), clamp(base_speed + corr)

corrector = DriveCorrector()


def align_to_front_wall(target=FRONT_STOP, timeout_steps=30):
    """Ustawia robota prostopadle do sciany z przodu w stalej odleglosci."""
    if not is_wall_in_front():
        return
    for _ in range(timeout_steps):
        if robot.step(timestep) == -1:
            return
        s0 = distance_sensors[0].getValue()
        s7 = distance_sensors[7].getValue()
        if min(s0, s7) < SIDE_WALL:
            break
        dist_err = ((s0 + s7) / 2.0 - target) * 0.01
        angle_err = (s0 - s7) * 0.01       # >0: prawy blizej -> obroc w prawo
        if abs(dist_err) < 0.05 and abs(angle_err) < 0.05:
            break
        left_motor.setVelocity(clamp(-dist_err + angle_err, -1.0, 1.0))
        right_motor.setVelocity(clamp(-dist_err - angle_err, -1.0, 1.0))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


def is_goal_reached():
    """Sprawdza czy robot wjechal na zielone pole koncowe"""
    vals = [s.getValue() for s in ground_sensors]
    # ciemniejsze pole zielone < 550
    return min(vals) < 550


def speed_for_remaining(remaining_m, max_speed=4.0, min_speed=1.8, ramp=0.04):
    """Predkosc kol [rad/s] zalezna od pozostalego dystansu [m]:
    pelna predkosc, a w ostatnich metrach liniowe hamowanie do min_speed."""
    if remaining_m <= 0:
        return 0.0
    v = max_speed * remaining_m / ramp
    return clamp(v, min_speed, max_speed)

# obrot w miejscu o 90 stopni
def turn_90_degrees(direction):
    target_wheel_rad = (math.pi * WHEELS_DIST) / (4.0 * WHEEL_RADIUS) * 1.12
    
    # pobranie pozycji poczatkowej z enkoderow kol
    start_left = left_encoder.getValue()
    start_right = right_encoder.getValue()
    
    turn_speed = 1.5
    if direction == 'right':
        left_motor.setVelocity(turn_speed)
        right_motor.setVelocity(-turn_speed)
    else:
        left_motor.setVelocity(-turn_speed)
        right_motor.setVelocity(turn_speed)
        
    while robot.step(timestep) != -1:
        current_left = left_encoder.getValue()
        current_right = right_encoder.getValue()
        
        # Mierzenie bezwzglednej zmiany pozycji
        delta_left = abs(current_left - start_left)
        delta_right = abs(current_right - start_right)
        avg_delta = (delta_left + delta_right) / 2.0
        
        if avg_delta >= target_wheel_rad:
            print(avg_delta)
            break
        
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


EDGE_OFFSET = 0.05   # [m] ile robot powinien dojechac zanim skreci

def step_one_cell_forward():
    corrector.reset()
    target_rad = 0.12 / WHEEL_RADIUS
    start_l = left_encoder.getValue()
    start_r = right_encoder.getValue()
    had_l, had_r = is_wall_on_left(), is_wall_on_right()
    stop_rad = target_rad
    edge_seen = False
    base_speed = 4.0

    while robot.step(timestep) != -1:
        ps = [s.getValue() for s in distance_sensors]
        front = max(ps[0], ps[7])
        left = ps[5]
        right = ps[2]

        print(f"front={front:.0f}  left={left:.0f}  right={right:.0f}")
        if is_goal_reached():
            return True
        if max(distance_sensors[0].getValue(), distance_sensors[7].getValue()) > FRONT_STOP:
            break

        travelled = (abs(left_encoder.getValue() - start_l) + abs(right_encoder.getValue() - start_r)) / 2.0

        if not edge_seen and ((had_l and not is_wall_on_left()) or (had_r and not is_wall_on_right())):
            edge_seen = True
            stop_rad = travelled + EDGE_OFFSET / WHEEL_RADIUS
            corrector.walls_lost = True            # brak sciany, jedz prosto bez korekty

        if travelled >= stop_rad:
            if edge_seen and is_wall_in_front():
                stop_rad = float("inf")
            else:
                break

        remaining_m = (stop_rad - travelled) * WHEEL_RADIUS
        base_speed = speed_for_remaining(remaining_m)
        vl, vr = corrector(base_speed)
        left_motor.setVelocity(vl)
        right_motor.setVelocity(vr)

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    return False

            
            
def right_hand_strategy():
    """Strategia prawej reki"""
    align_to_front_wall()
    if not is_wall_on_right():
        turn_90_degrees("right")
        step_one_cell_forward()
    elif not is_wall_in_front():
        step_one_cell_forward()
    elif not is_wall_on_left():
        turn_90_degrees("left")
        step_one_cell_forward()
    else:
        turn_90_degrees("right")
        turn_90_degrees("right")


def left_hand_strategy():
    """Strategia lewej reki"""
    align_to_front_wall()
    if not is_wall_on_left():
        turn_90_degrees("left")
        step_one_cell_forward()
    elif not is_wall_in_front():
        step_one_cell_forward()
    elif not is_wall_on_right():
        turn_90_degrees("right")
        step_one_cell_forward()
    else:
        turn_90_degrees("left")
        turn_90_degrees("left")

# --- Main loop ---------------------------------------------------------------
while robot.step(timestep) != -1:
#   if is_goal_reached():
#     print("META OSIĄGNIĘTA!")
#     left_motor.setVelocity(0.0)
#     right_motor.setVelocity(0.0)
#     break

    # strategia testowana
    right_hand_strategy()
    

    # Uncomment to see the sensor values in the console:
    # TODO: decide how the robot should move.
    # Right now it drives forward and simply stops in front of a wall.
