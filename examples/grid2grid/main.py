from evasdk import Eva
from evaUtilities import EvaGrids
from config.config_manager import load_use_case_config


if __name__ == "__main__":
    # Load use-case parameters
    config = load_use_case_config()

    # Connection to robot
    host = config['EVA']['comm']['host']
    token = config['EVA']['comm']['token']
    eva = Eva(host, token)

    # Compute grid points and robot joints
    eva_box = EvaGrids(eva, config, show_plot=True)
    joints = eva_box.get_grid_points(config['grids']['names'])

    # Go home before starting
    with eva.lock():
        eva.control_go_to(config['EVA']['home'])

    while True:
        for counter in range(len(joints[(config['grids']['names'][0])]['pick'])):
            joints_home = config['EVA']['home']
            joints_pick = joints[config['grids']['names'][0]]['pick'][counter]
            joints_drop = joints[config['grids']['names'][1]]['pick'][counter]
            joints_pick_hover = joints[config['grids']['names'][0]]['hover'][counter]
            joints_drop_hover = joints[config['grids']['names'][1]]['hover'][counter]

            # USER DEFINED WAY-POINTS
            joints_operation_A = []
            joints_operation_B = []
            joints_operation_C = []

            tool_path_grid_to_grid = {
                "metadata": {
                    "version": 2,
                    "default_max_speed": 0.1,
                    "next_label_id": 7,
                    "payload": config['EVA']['end_effector']['payload'],
                    "analog_modes": {"i0": "voltage", "i1": "voltage", "o0": "voltage", "o1": "voltage"}
                    },
                "waypoints": [
                    {"label_id": 1, "joints": joints_home},
                    {"label_id": 2, "joints": joints_pick_hover},
                    {"label_id": 3, "joints": joints_pick},
                    {"label_id": 4, "joints": joints_operation_A},
                    {"label_id": 5, "joints": joints_drop_hover},
                    {"label_id": 6, "joints": joints_drop},
                ],
                "timeline": [
                    {"type": "home", "waypoint_id": 0},
                    {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 1},
                    {"type": "output-set", "io": {"location": "base", "type": "digital", "index": 0}, "value": True},
                    {"type": "trajectory", "trajectory": "linear", "waypoint_id": 2},
                    {"type": "wait", "condition": {"type": "time", "duration": 0.2}},
                    {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 1},
                    {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 3},
                    {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 4},
                    {"type": "trajectory", "trajectory": "linear", "waypoint_id": 5},
                    {"type": "output-set", "io": {"location": "base", "type": "digital", "index": 0}, "value": False},
                    {"type": "trajectory", "trajectory": "linear", "waypoint_id": 4},
                    {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 0},
                ]
            }

            with eva.lock():
                eva.control_wait_for_ready()
                eva.toolpaths_use(tool_path_grid_to_grid)
                eva.control_run(loop=1, mode="automatic")
