import numpy as np
from fabrics.planner.serialized_planner import SerializedFabricPlanner

if __name__ == "__main__":
    planner = SerializedFabricPlanner(
        "controller.pbz2"
    )
    acceleration = planner.compute_action(
        q=np.array([0.1, 0.5, 0.1, 0.1, 0.1, 0.1, 0.1]),
        qdot=np.array([0.4, 1, 0.1, 0.1, 0.1, 0.1, 0.1]),
        weight_goal_0=0.1,
        x_goal_0=np.array([0.3, 0.2, 0.1, 0.2, 0.2, 0.3, 0.1]),
    )
    print(acceleration)
