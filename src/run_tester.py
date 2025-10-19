#only I test by this 
from src.models import CarPose, Cone
from src.tester import PathTester

# --- Define the car pose (at origin, facing right) ---
car_pose = CarPose(x=0.0, y=0.0, yaw=0.0)

# --- Define two cones: one blue (left) and one yellow (right) ---
cones = [
    Cone(x=3.0, y=1.5, color=1),  # Blue (Left)
    Cone(x=3.0, y=-1.5, color=0), # Yellow (Right)
]

# --- Run the tester ---
tester = PathTester(cones, car_pose)
tester.run()
