from flask import Flask, render_template, request
import socket
# from obj_def import OmniRobot, Obstacles, VisionData, Obstacle
# from trajectory_functions import check_distances, Obstacle_Avoidance_Control, Trajectory_Control


app = Flask(__name__)

# Global object initialization for control
# omni_robot = OmniRobot()
# obstacles = Obstacles()
# vision_data = VisionData()
# ball = Obstacle(0, 0)

# Inicialización de variables
robot_info = [0.0, 0.0, 0.0]  # [PosX, PosY, Angle]
ball_info = [0.0, 0.0]        # [PosX, PosY]
obstacles_matrix = [[], []]   # 2 filas: [PosX_list, PosY_list], columnas = obstáculos

# UDP Configuration
# Replace with your actual UDP server IP and port
UDP_IP = "10.0.0.107"  #192.168.20.7
UDP_PORT = 12345           

@app.route('/')
def index():
    return render_template('index.html')

def send_velocity_to_robot(velocity_body_setpoints):
    """
    Envía los datos de velocidad al robot por UDP.
    Retorna un mensaje de éxito o error.
    """
    try:
        msg = f"VEL,{velocity_body_setpoints[0]},{velocity_body_setpoints[1]},{velocity_body_setpoints[2]}"
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
        return f"Comando procesado y enviado: {msg}"
    except Exception as e:
        return f"Error enviando velocidades: {str(e)}"

@app.route('/send', methods=['POST'])
def send():
    global robot_info, ball_info, obstacles_matrix
    # Solo acepta JSON
    data = request.get_json()
    try:
        robot = data['robot']
        ball = data['ball']
        dummies = data.get('dummies', [])


        robot_info[0] = float(robot['x'])
        robot_info[1] = float(robot['y'])
        robot_info[2] = float(robot['theta'])

        ball_info[0] = float(ball['x'])
        ball_info[1] = float(ball['y'])

        obstacles_matrix[0] = [float(d['x']) for d in dummies]
        obstacles_matrix[1] = [float(d['y']) for d in dummies]

        print(f"robot_info almacenado: {robot_info}")
        print(f"ball_info almacenado: {ball_info}")
        print(f"obstacles_matrix almacenado: {obstacles_matrix}")

        # Enviar comando UDP usando send_velocity_to_robot
        # Puedes ajustar los valores enviados según tu lógica de control
        # Aquí se envía la velocidad como ejemplo, puedes cambiarlo por lo que necesites
        # Ejemplo: enviar [vx, vy, omega] = [0, 0, 0] (modifica según tu lógica)
        velocity_body_setpoints = data.get('velocity', [0.0, 0.0, 0.0])
        udp_result = send_velocity_to_robot(velocity_body_setpoints)

        return (
            f"Datos escritos: robot_info={robot_info}, "
            f"ball_info={ball_info}, "
            f"obstacles_matrix={obstacles_matrix}, "
            f"UDP: {udp_result}"
        )
    except Exception as e:
        return f"Error procesando JSON: {str(e)}", 400

def get_current_vision_data():
    """
    Retorna los datos actuales de robot, ball y obstacles.
    """
    return robot_info, ball_info, obstacles_matrix

