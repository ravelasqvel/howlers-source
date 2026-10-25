from app import get_current_vision_data

class OmniRobot:
    def __init__(self):
        self.PosX = 0.0
        self.PosY = 0.0
        self.Angle = 0.0
        self.VbX = 0.0
        self.VbY = 0.0
        self.Vomega = 0.0
        # Add more attributes as needed

    def update(self, vision_data):
        # Update position and angle using vision_data
        if vision_data is not None and vision_data.robot_info:
            self.PosX = vision_data.robot_info[0]
            self.PosY = vision_data.robot_info[1]
            self.Angle = vision_data.robot_info[2]

class Obstacle:
    def __init__(self, PosX=0.0, PosY=0.0):
        self.PosX = PosX
        self.PosY = PosY
        # Add more attributes as needed

    def update(self, *args):
        """
        Can receive:
        - update(PosX, PosY)
        - update(vision_data) -> takes the ball position from vision_data.ball_info
        """
        if len(args) == 2:
            self.PosX, self.PosY = args
        elif len(args) == 1:
            vision_data = args[0]
            if vision_data is not None and vision_data.ball_info:
                self.PosX = vision_data.ball_info[0]
                self.PosY = vision_data.ball_info[1]

class Obstacles:
    def __init__(self):
        self.list = []
        # Add more attributes as needed

    def update(self, vision_data):
        # Update all obstacles using vision_data
        if vision_data is not None and vision_data.obstacles_matrix is not None:
            num_vision_obstacles = len(vision_data.obstacles_matrix[0])
            num_current_obstacles = len(self.list)
            # Update existing ones
            for idx in range(min(num_current_obstacles, num_vision_obstacles)):
                posx = vision_data.obstacles_matrix[0][idx]
                posy = vision_data.obstacles_matrix[1][idx]
                self.list[idx].update(posx, posy)
            # Add new ones if there are more in the vision matrix
            for idx in range(num_current_obstacles, num_vision_obstacles):
                posx = vision_data.obstacles_matrix[0][idx]
                posy = vision_data.obstacles_matrix[1][idx]
                self.list.append(Obstacle(posx, posy))

class VisionData:
    def __init__(self):
        # [PosX, PosY, Angle]
        self.robot_info = [0.0, 0.0, 0.0]
        # [PosX, PosY]
        self.ball_info = [0.0, 0.0]
        # Matriz: 2 rows (PosX, PosY), columnas = obstacles
        self.obstacles_matrix = [[], []]
        # Add more attributes as needed

    def Vision_Processing(self):
        # Actualiza los datos usando get_current_vision_data de app.py
        robot, ball, obstacles = get_current_vision_data()
        self.robot_info = list(robot)
        self.ball_info = list(ball)
        self.obstacles_matrix = [list(obstacles[0]), list(obstacles[1])]