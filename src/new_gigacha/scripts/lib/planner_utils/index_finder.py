
from math import hypot

class IndexFinder:
    def __init__(self, ego):
        self.ego = ego
        self.save_idx = 0

    def run(self):
        

        min_dis = -1
        min_idx = 0

        if self.ego.mission = "parking":
            path = self.ego.local_path
        else:
            path = self.ego.global_path

        print(f"Global path length: {len(path.x)}")
        step_size = 100

        for i in range(max(self.ego.index - step_size, 0), self.ego.index + step_size):
            try:
                dis = hypot(path.x[i] - self.ego.pose.x, path.y[i] - self.ego.pose.y)
            except IndexError:
                break
            if (min_dis > dis or min_dis == -1) and self.save_idx <= i:
                min_dis = dis
                min_idx = i
                self.save_idx = i


        self.ego.index = min_idx
