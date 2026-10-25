from app import *
import math

# Define the value of the external macro
OBSTACLE_MIN_DIST = 0.5  # example: 0.5 meters
BALL_MIN_DIST = 0.3      # example: 0.3 meters

def euclidean_distance(obj1, obj2):
    """
    Calculates the Euclidean distance between two objects with PosX and PosY attributes
    """
    return math.hypot(obj1.PosX - obj2.PosX, obj1.PosY - obj2.PosY)

def check_distances(omni_robot, obstacles, ball):
    """
    Calculates the Euclidean distance to each obstacle and to the ball.
    Returns:
    - is_ball_close: True if the ball is within BALL_MIN_DIST range
    - is_obstacle_close: True if any obstacle is within OBSTACLE_MIN_DIST range
    - obj_obstacle: the closest obstacle if within range, otherwise None
    """
    # Obstacles
    min_dist = float('inf')
    closest_obstacle = None

    for obstacle in obstacles.list:
        dist = euclidean_distance(omni_robot, obstacle)
        if dist < min_dist:
            min_dist = dist
            closest_obstacle = obstacle

    if closest_obstacle is not None and min_dist < OBSTACLE_MIN_DIST:
        is_obstacle_close = True
        obj_obstacle = closest_obstacle
    else:
        is_obstacle_close = False
        obj_obstacle = None

    # Ball
    is_ball_close = False
    if ball is not None:
        ball_dist = euclidean_distance(omni_robot, ball)
        if ball_dist < BALL_MIN_DIST:
            is_ball_close = True

    return is_ball_close, is_obstacle_close, obj_obstacle

def Obstacle_Avoidance_Control(omni_robot, obj_obstacle):
    """
    Prototype function for obstacle avoidance control.
    Returns: [VbX, VbY, Vomega] (velocities in the robot's body frame)
    """
    # Obstacle avoidance logic here
    VbX = 0.0
    VbY = 0.0
    Vomega = 0.0
    return [VbX, VbY, Vomega]

def Trajectory_Control(desired_position_omnirobot, actual_position_omnirobot):
    """
    Prototype function for trajectory control.
    Returns: [VbX, VbY, Vomega] (velocities in the robot's body frame)
    """
    # Trajectory control logic here
    VbX = 0.0
    VbY = 0.0
    Vomega = 0.0
    return [VbX, VbY, Vomega]

def OmniRobot_Send_Velocity_Setpoints(velocity_body_setpoints):
    """
    Envía los setpoints de velocidad al robot usando la función de app.py.
    velocity_body_setpoints: lista o tupla con (VbX, VbY, Vomega)
    Retorna el mensaje de éxito o error de la función de envío.
    """
    # Validación de datos
    if not isinstance(velocity_body_setpoints, (list, tuple)):
        return "Error: velocity_body_setpoints debe ser una lista o tupla."
    if len(velocity_body_setpoints) != 3:
        return "Error: velocity_body_setpoints debe tener exactamente 3 elementos."
    if not all(isinstance(v, (int, float)) for v in velocity_body_setpoints):
        return "Error: Todos los elementos de velocity_body_setpoints deben ser numéricos."

    return send_velocity_to_robot(velocity_body_setpoints)