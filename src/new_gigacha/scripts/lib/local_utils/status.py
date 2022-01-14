class Status:
    def __init__(self):
        self.gps = -1 # reliability of gps
        self.imu = -1 # reliability of imu
        self.encoder = 0 # direction of vehicle {forward : 1, backward : -1}