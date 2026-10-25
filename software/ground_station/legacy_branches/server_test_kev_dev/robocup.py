from obj_def import *
from trajectory_functions import *
from app import *
import time

def init_objects():
    omni_robot = OmniRobot()
    obstacles = Obstacles()
    vision_data = VisionData()
    ball = Obstacle(0,0)
    return omni_robot, obstacles, vision_data, ball

def periodic_thread(omni_robot, obstacles, vision_data, ball):
    vision_data.Vision_Processing()  # Calls the method of the VisionData class
    omni_robot.update(vision_data)
    obstacles.update(vision_data)
    ball.update(vision_data)

    is_ball_close, is_obstacle_close, obj_obstacle = check_distances(omni_robot, obstacles, ball)
    desired_position_omnirobot = [0, 0]
    actual_position_omnirobot = [0, 0]

    # If the ball is NOT close, go towards the ball
    if not is_ball_close:
        desired_position_omnirobot = [ball.PosX, ball.PosY]
    else:
        # Here goes the logic when the ball is close
        pass

    if is_obstacle_close:
        velocity_body_setpoints = Obstacle_Avoidance_Control(omni_robot, obj_obstacle)
    else:
        actual_position_omnirobot = [omni_robot.PosX, omni_robot.PosY]
        velocity_body_setpoints = Trajectory_Control(desired_position_omnirobot, actual_position_omnirobot)

    OmniRobot_Send_Velocity_Setpoints(velocity_body_setpoints)




import threading
from app import app

def run_flask():
    print("Starting Flask server...")
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
    print("Flask server started at http://localhost:5000/")

if __name__ == '__main__':
    # Iniciar Flask en un hilo separado
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    # Object initialization
    omni_robot, obstacles, vision_data, ball = init_objects()
    while True:
        periodic_thread(omni_robot, obstacles, vision_data, ball)
        time.sleep(0.1) # change for an exact cycle time measurement if necessary