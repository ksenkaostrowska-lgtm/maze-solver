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
from collections import deque
from controller import Robot

# parametry robota
MAX_SPEED = 6.28
WHEEL_RADIUS = 0.0205
WHEELS_DIST = 0.052
WALL_CLOSE = 100
FRONT_STOP = 150.0
KP_BOTH = 0.001
KP_ONE = 0.0015
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
    return (distance_sensors[5].getValue() > WALL_CLOSE)


class DriveCorrector:
    def __init__(self):
        self.reset()

    def reset(self):
        self.mode = None
        self.ref = None
        self.last_corr = 0.0
        self.walls_lost = False

    def __call__(self, base_speed):
        left = distance_sensors[5].getValue()
        right = distance_sensors[2].getValue()
        has_l = is_wall_on_left()
        has_r = is_wall_on_right()

        if has_l and has_r:
            mode = "both"
        elif has_l:
            mode = "left"
        elif has_r:
            mode = "right"
        else:
            mode = "enc"

        # mode sie zmienil czyli ubyla lub przybyla sciana
        if self.mode is not None and mode != self.mode:
            prev_walls = {"both": 2, "left": 1, "right": 1, "enc": 0}
            if prev_walls[mode] < prev_walls[self.mode]:
                self.walls_lost = True
            # przy przelaczeniu mode zapamietuje aktualny odczyt jako setpoint
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
            # jesli nie ma scian, korekta w oparciu o enkodery kol
            dl = left_encoder.getValue() - self.ref[0]
            dr = right_encoder.getValue() - self.ref[1]
            error = (dl - dr) * KP_ENC

        corr = clamp(error, -MAX_CORR, MAX_CORR)
        # max zmiana 0.05 rad/s na krok
        corr = clamp(corr, self.last_corr - 0.05, self.last_corr + 0.05)
        self.last_corr = corr
        vl = base_speed - corr
        vr = base_speed + corr
        # przesun oba kola w dol o nadwyzke, zeby zachowac roznice predkosci
        excess = max(vl, vr) - MAX_SPEED
        if excess > 0:
            vl -= excess
            vr -= excess
        return clamp(vl), clamp(vr)

corrector = DriveCorrector()


def align_to_front_wall(target=FRONT_STOP, timeout_steps=50):
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
    stop_motors()


def is_goal_reached():
    """Sprawdza czy robot wjechal na zielone pole koncowe"""
    vals = [s.getValue() for s in ground_sensors]
    # ciemniejsze pole zielone < 600
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
    corr = 1.12
    target_wheel_rad = (math.pi * WHEELS_DIST) / (4.0 * WHEEL_RADIUS) * corr
    
    # pobranie pozycji poczatkowej z enkoderow kol
    start_left = left_encoder.getValue()
    start_right = right_encoder.getValue()
    
    turn_speed = 3.5
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
            break
        
    stop_motors()


EDGE_OFFSET = 0.05   # [m] ile robot powinien dojechac zanim skreci

def step_one_cell_forward():
    corrector.reset()
    target_rad = 0.12 / WHEEL_RADIUS
    start_l = left_encoder.getValue()
    start_r = right_encoder.getValue()
    had_l, had_r = is_wall_on_left(), is_wall_on_right()
    stop_rad = target_rad
    edge_seen = False
    base_speed = MAX_SPEED
    remaining_speed = base_speed
    goal_seen = False

    while robot.step(timestep) != -1:
        if is_goal_reached():
            goal_seen = True

        if max(distance_sensors[0].getValue(), distance_sensors[7].getValue()) > FRONT_STOP:
            break

        travelled = (abs(left_encoder.getValue() - start_l) + abs(right_encoder.getValue() - start_r)) / 2.0

        if not edge_seen and ((had_l and not is_wall_on_left()) or (had_r and not is_wall_on_right())):
            edge_seen = True
            stop_rad = travelled + EDGE_OFFSET / WHEEL_RADIUS
            remaining_speed = speed_for_remaining((stop_rad - travelled) * WHEEL_RADIUS)
            corrector.walls_lost = True            # brak sciany - jedz prosto bez korekty

        if travelled >= stop_rad:
            if edge_seen and is_wall_in_front():
                stop_rad = float("inf")
            else:
                break

        base_speed = remaining_speed
        vl, vr = corrector(base_speed)
        left_motor.setVelocity(vl)
        right_motor.setVelocity(vr)

    stop_motors()
    return goal_seen
 
            
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


def stop_motors():
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


# Flood fill


N, E, S, W = 0, 1, 2, 3
DX = (0, 1, 0, -1)
DY = (1, 0, -1, 0)
MAX_SIZE = 16

MAZE_PARAMS = {
    "maze_1":{
        "w": 5,
        "h": 5,
        "start": (0, 0),
        "start_heading": N,
        "goal": (4, 4)
    },
    "maze_2":{
        "w": 8,
        "h": 8,
        "start": (0, 0),
        "start_heading": N,
        "goal": (7, 7)
    }
}


class SimpleMazeMap:
    """Mapa ścian oparta na słowniku, zna wspolrzedne pkt koncowego i wymiary"""

    def __init__(self, maze_name):
        params = MAZE_PARAMS[maze_name]
        self.w, self.h = params["w"], params["h"]
        self.goal_cell = params["goal"]
        self.heading = params["start_heading"]
        # zbior kierunkow dla kazdego pola, gdzie sa sciany
        self.walls = set()

    def set_wall(self, x, y, d):
        """Zapisuje ścianę w danej kratce oraz u sąsiada"""
        self.walls.add((x, y, d))
        self.walls.add((x + DX[d], y + DY[d], (d + 2) % 4))

    def note_open(self, x, y, d):
        pass    # wymiary są stałe więc nie powiększam

    def blocked(self, x, y, d):
        nx, ny = x + DX[d], y + DY[d]
        if not (0 <= nx < self.w and 0 <= ny < self.h):
            return True
        return (x, y, d) in self.walls

    def flood(self):
        dist = {self.goal_cell: 0}
        q = deque([self.goal_cell])
        while q:
            x, y = q.popleft()
            for d in range(4):
                if self.blocked(x, y, d):
                    continue
                n = (x + DX[d], y + DY[d])
                if n not in dist:
                    dist[n] = dist[(x, y)] + 1
                    q.append(n)
        return dist

class DynamicMazeMap:
    """Mapa bez znanych wymiarów; zakłada kwadrat i metę w przeciwległym rogu."""

    def __init__(self):
        self.walls = set()
        self.size = 2

    def set_wall(self, x, y, d):
        self.walls.add((x, y, d))
        self.walls.add((x + DX[d], y + DY[d], (d + 2) % 4))

    def note_open(self, x, y, d):
        # sprawdzam, czy kratki wychodza poza założony size
        nx, ny = x + DX[d], y + DY[d]
        self.size = max(self.size, nx + 1, ny + 1)

    def blocked(self, x, y, d):
        nx, ny = x + DX[d], y + DY[d]
        if not (0 <= nx < self.size and 0 <= ny < self.size):
            return True
        return (x, y, d) in self.walls

    def goal(self):
        return (self.size - 1, self.size - 1)

    def flood(self):
        g = self.goal()
        dist = {g: 0}
        q = deque([g])
        while q:
            x, y = q.popleft()
            for d in range(4):
                if self.blocked(x, y, d):
                    continue
                n = (x + DX[d], y + DY[d])
                if n not in dist:
                    dist[n] = dist[(x, y)] + 1
                    q.append(n)
        return dist


def sense_walls(maze, pos_x, pos_y, heading):
    """Zapisuje ściany i otwarte przejścia wokół robota"""
    front_dir = heading
    right_dir = (heading + 1) % 4
    left_dir = (heading + 3) % 4

    if is_wall_in_front():
        maze.set_wall(pos_x, pos_y, front_dir)
    else:
        maze.note_open(pos_x, pos_y, front_dir)

    if is_wall_on_right():
        maze.set_wall(pos_x, pos_y, right_dir)
    else:
        maze.note_open(pos_x, pos_y, right_dir)

    if is_wall_on_left():
        maze.set_wall(pos_x, pos_y, left_dir)
    else:
        maze.note_open(pos_x, pos_y, left_dir)


def turn_to(heading, new_heading):
    """Obraca robota i zwraca nowy kierunek."""
    diff = (new_heading - heading) % 4
    if diff == 1:
        turn_90_degrees("right")
    elif diff == 3:
        turn_90_degrees("left")
    elif diff == 2:
        turn_90_degrees("right")
        turn_90_degrees("right")
    return new_heading


def choose_direction(maze, pos_x, pos_y, heading, dist):
    """Wybiera sąsiada o najmniejszej odległości do mety."""
    if (pos_x, pos_y) not in dist:
        return None
    best_dir, best_val = None, dist[(pos_x, pos_y)]
    for d in (heading, (heading + 1) % 4, (heading + 3) % 4, (heading + 2) % 4):
        if maze.blocked(pos_x, pos_y, d):
            continue
        val = dist.get((pos_x + DX[d], pos_y + DY[d]))
        if val is not None and val < best_val:
            best_dir, best_val = d, val
    return best_dir

# STRATEGIE

def run_right_hand():
    while robot.step(timestep) != -1:
        if is_goal_reached():
            print("META OSIĄGNIĘTA!")
            stop_motors()
            return
        right_hand_strategy()

def run_left_hand():
    while robot.step(timestep) != -1:
        if is_goal_reached():
            print("META OSIĄGNIĘTA!")
            stop_motors()
            return
        left_hand_strategy()


def run_simple_flood_fill(maze_name):
    maze = SimpleMazeMap(maze_name)
    pos_x, pos_y = 0, 0
    heading = maze.heading
    goal = maze.goal_cell

    while robot.step(timestep) != -1:

        align_to_front_wall()
        sense_walls(maze, pos_x, pos_y, heading)
        dist = maze.flood()

        best = choose_direction(maze, pos_x, pos_y, heading, dist)
        if best is None:
            print("Brak drogi - przechodzę na regułę prawej ręki")
            run_right_hand()
            return

        heading = turn_to(heading, best)
        if is_wall_in_front():
            maze.set_wall(pos_x, pos_y, heading)
            continue

        if (pos_x, pos_y) == goal or step_one_cell_forward():
            print("META OSIĄGNIĘTA")
            stop_motors()
            return
        
        pos_x += DX[heading]
        pos_y += DY[heading]


def run_dynamic_flood_fill():
    maze = DynamicMazeMap()
    pos_x, pos_y = 0, 0
    heading = N

    while robot.step(timestep) != -1:
        vals = [s.getValue() for s in ground_sensors]

        align_to_front_wall()
        sense_walls(maze, pos_x, pos_y, heading)

        if (pos_x, pos_y) == maze.goal():
            maze.size += 1

        dist = maze.flood()
        while (pos_x, pos_y) not in dist and maze.size < MAX_SIZE:
            maze.size += 1
            dist = maze.flood()

        best = choose_direction(maze, pos_x, pos_y, heading, dist)
        if best is None:
            print("Brak drogi - przechodzę na regułę prawej ręki")
            run_right_hand()
            return

        heading = turn_to(heading, best)
        if is_wall_in_front():
            maze.set_wall(pos_x, pos_y, heading)
            continue

        if step_one_cell_forward():
            print("META OSIĄGNIĘTA")
            stop_motors()
            return
        pos_x += DX[heading]
        pos_y += DY[heading]


run_dynamic_flood_fill()
# run_simple_flood_fill("maze_1")
# run_right_hand()
# run_left_hand()
