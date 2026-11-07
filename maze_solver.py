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
WALL_CLOSE = 80
SIDE_WALL = 75
SIDE_SETPOINT = 110
KP_BOTH = 0.003
KP_ONE = 0.004
KP_ENC = 2
MAX_CORR = 0.5

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


def drive_correction(base_speed, start_l, start_r):
    """Zwraca (v_left, v_right) tak, by robot jechał środkiem korytarza.
    Tryby: dwie ściany -> centrowanie; jedna ściana -> trzymaj stały dystans;
    brak ścian -> jedź prosto po enkoderach."""
    left = distance_sensors[5].getValue()
    right = distance_sensors[2].getValue()

    if left > SIDE_WALL and right > SIDE_WALL:
        error = (left - right) * KP_BOTH
    elif left > SIDE_WALL:
        error = (left - SIDE_SETPOINT) * KP_ONE      # za blisko lewej -> dodatnie -> skręć w prawo
    elif right > SIDE_WALL:
        error = (SIDE_SETPOINT - right) * KP_ONE     # za blisko prawej -> ujemne -> skręć w lewo
    else:
        # brak ścian: wyrównuj drogę obu kół, żeby nie skręcać
        dl = left_encoder.getValue() - start_l
        dr = right_encoder.getValue() - start_r
        error = (dl - dr) * KP_ENC

    corr = clamp(error, -MAX_CORR, MAX_CORR)
    return clamp(base_speed - corr), clamp(base_speed + corr)

def align_to_front_wall(target=150.0, timeout_steps=60):
    """Ustawia robota prostopadle do ściany z przodu i w stałej odległości."""
    for _ in range(timeout_steps):
        if robot.step(timestep) == -1:
            return
        f0 = distance_sensors[0].getValue()
        f7 = distance_sensors[7].getValue()
        if f0 < SIDE_WALL and f7 < SIDE_WALL:
            break                                  # nie ma ściany, nie ma do czego równać
        dist_err = ((f0 + f7) / 2.0 - target) * 0.01   # za blisko -> cofnij
        angle_err = (f0 - f7) * 0.01                   # różnica = skos
        if abs(dist_err) < 0.05 and abs(angle_err) < 0.05:
            break
        left_motor.setVelocity(clamp(-dist_err - angle_err, -1.0, 1.0))
        right_motor.setVelocity(clamp(-dist_err + angle_err, -1.0, 1.0))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


def is_goal_reached():
  """Sprawdza czy robot wjechał na zielone pole końcowe (ciemniejsze od podłogi)."""
  vals = [s.getValue() for s in ground_sensors]
  # Podłoga ma zwykle > 700, ciemniejsze pole zielone < 550
  return min(vals) < 550

# obrót w miejscu o 90 stopni
def turn_90_degrees(direction):
    target_wheel_rad = (math.pi * WHEELS_DIST) / (4.0 * WHEEL_RADIUS) * 1.12
    
    # pobranie pozycji początkowej z enkoderów kół
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
        print(f"left={current_left:.2f}  right={current_right:.2f}")
        
        # Mierzenie bezwzględnej zmiany pozycji
        delta_left = abs(current_left - start_left)
        delta_right = abs(current_right - start_right)
        avg_delta = (delta_left + delta_right) / 2.0
        
        if avg_delta >= target_wheel_rad:
            print(avg_delta)
            break
        
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


def step_one_cell_forward():
  TARGET_DIST = 0.12
  target_rad = TARGET_DIST / WHEEL_RADIUS
  start_l = left_encoder.getValue()
  start_r = right_encoder.getValue()
  base_speed = 3.0

  while robot.step(timestep) != -1:
    if is_goal_reached():
      return True

    if is_wall_in_front():
      break

    dl = abs(left_encoder.getValue() - start_l)
    dr = abs(right_encoder.getValue() - start_r)
    if (dl + dr) / 2.0 >= target_rad:
      break

    vl, vr = drive_correction(base_speed, start_l, start_r)
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)

  left_motor.setVelocity(0.0)
  right_motor.setVelocity(0.0)
  return False

            
            
def right_hand_strategy():
  """Strategia prawej ręki: faworyzuje prawą stronę."""
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
  """Strategia lewej ręki: faworyzuje lewą stronę."""
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
  if is_goal_reached():
    print("META OSIĄGNIĘTA!")
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    break

  # Tutaj wybierasz strategię, którą chcesz testować:
  right_hand_strategy()
    

    # Uncomment to see the sensor values in the console:
    # print(f"front={front:.0f}  left={left:.0f}  right={right:.0f}")
    # TODO: decide how the robot should move.
    # Right now it drives forward and simply stops in front of a wall.

