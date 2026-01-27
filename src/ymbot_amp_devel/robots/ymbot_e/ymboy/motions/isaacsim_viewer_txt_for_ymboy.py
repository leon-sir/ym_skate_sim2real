import argparse
import json
import os 

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(
    description="Visualize G1 robot by directly setting joint positions from motion data (txt format)."
)
AppLauncher.add_app_launcher_args(parser)
parser.add_argument(
    "--file_path",
    type=str,
    default="walk1_subject1_clipped_2700_3600.txt",
    help="Path to the motion data file (.txt). Can be absolute or relative."
)
parser.add_argument(
    "--slow_down_factor",
    type=float,
    default=1.0,
    help="Playback speed multiplier (<1 for slower, >1 for faster)."
)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.actuators import ImplicitActuatorCfg
from legged_lab.assets.ymbot_e.ymbot import Ymboy_CFG
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

# Fixed settings
NUM_ENVS = 1


class G1MotionLoader:
    """Simple motion loader for G1 txt format data"""
    
    def __init__(self, file_path, device):
        self.device = device
        
        # Load motion data from txt file
        with open(file_path, 'r') as f:
            data = json.load(f)
        
        self.frames = torch.tensor(data["Frames"], device=device, dtype=torch.float32)
        self.frame_duration = data["FrameDuration"]
        self.num_frames = len(self.frames)
        
        print(f"Loaded {self.num_frames} frames with duration {self.frame_duration}")
        print(f"Each frame has {self.frames.shape[1]} elements")
        
        self.motion_joint_names = [
            # 腰部关节 (1个)
            "waist_yaw_joint",
            # 右臂关节 (5个)
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_yaw_joint",
            # 左臂关节 (5个)
            "left_shoulder_pitch_joint", 
            "left_shoulder_roll_joint", 
            "left_shoulder_yaw_joint",
            "left_elbow_joint", 
            "left_wrist_yaw_joint",
            # 右腿关节 (6个)
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            # 左腿关节 (6个)
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
        ]
        
    def create_joint_mapping(self, robot_joint_names):
        """Create mapping from motion data indices to robot joint indices"""
        self.joint_mapping = {}
        for motion_idx, motion_joint_name in enumerate(self.motion_joint_names):
            if motion_joint_name in robot_joint_names:
                robot_idx = robot_joint_names.index(motion_joint_name)
                self.joint_mapping[motion_idx] = robot_idx
            else:
                print(f"Warning: Motion joint '{motion_joint_name}' not found in robot joints")
        
        print(f"Created joint mapping for {len(self.joint_mapping)} joints:")
        for motion_idx, robot_idx in self.joint_mapping.items():
            print(f"  Motion[{motion_idx}] '{self.motion_joint_names[motion_idx]}' -> Robot[{robot_idx}]")
        
    def get_frame_at_time(self, time):
        """Get frame data at specific time"""
        frame_idx = int(time / self.frame_duration) % self.num_frames
        return self.frames[frame_idx]
    
    def get_joint_positions(self, frame_data):
        """Extract joint positions from frame data (first 29 elements)"""
        return frame_data[:23]
    
    def get_joint_velocities(self, frame_data):
        """Extract joint velocities from frame data (elements 29-57)"""
        return frame_data[23:46]
    
    def get_end_effector_positions(self, frame_data):
        """Extract end effector positions from frame data (last 12 elements)"""
        # Order: left_hand(3), right_hand(3), left_foot(3), right_foot(3)
        return frame_data[46:58].reshape(4, 3)


class G1SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", 
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )
    # Disable gravity and actuation so the robot is purely kinematic
    Robot = Ymboy_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=Ymboy_CFG.spawn.usd_path,
            activate_contact_sensors=Ymboy_CFG.spawn.activate_contact_sensors,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=3.0,
                max_angular_velocity=3.0,
                max_depenetration_velocity=10.0,
            ),
            articulation_props=Ymboy_CFG.spawn.articulation_props 
        ),
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=Ymboy_CFG.actuators["legs"].joint_names_expr,
                stiffness=0.0,
                damping=0.0,
                effort_limit=0.0
            ),
            "feet": ImplicitActuatorCfg(
                joint_names_expr=Ymboy_CFG.actuators["feet"].joint_names_expr,
                stiffness=0.0,
                damping=0.0,
                effort_limit=0.0
            ),
            "shoulders": ImplicitActuatorCfg(
                joint_names_expr=Ymboy_CFG.actuators["shoulders"].joint_names_expr,
                stiffness=0.0,
                damping=0.0,
                effort_limit=0.0
            ),
            "arms": ImplicitActuatorCfg(
                joint_names_expr=Ymboy_CFG.actuators["arms"].joint_names_expr,
                stiffness=0.0,
                damping=0.0,
                effort_limit=0.0
            )
        },
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.8),  # G1 default height
            joint_pos={".*": 0.0}
        )
    )


# Create coordinate-frame markers for end effectors
def create_coordinate_markers() -> VisualizationMarkers:
    marker_cfg = VisualizationMarkersCfg(
        prim_path="/Visuals/EndEffectorFrames",
        markers={
            "frame": sim_utils.UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/frame_prim.usd",
                scale=(0.1, 0.1, 0.1),
                visual_material=None  # Use the original colors inside the USD
            )
        }
    )
    return VisualizationMarkers(marker_cfg)


# Draw coordinate frames for end effectors
def draw_end_effector_frames(
    scene: InteractiveScene, 
    markers: VisualizationMarkers,
    end_effector_positions: torch.Tensor,  # Shape: (4, 3) - world positions
    env_origins: torch.Tensor
):
    """Draw coordinate frames for the end effectors."""
    # end_effector_positions already in world coordinates, no need to add env_origins again
    world_positions = end_effector_positions
    
    # Create identity orientations for all end effectors
    orientations = torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=scene.device).repeat(4, 1)
    
    markers.visualize(
        world_positions,
        orientations,
        marker_indices=torch.tensor([0] * 4, device=scene.device)
    )


def run_simulator(
    sim: sim_utils.SimulationContext,
    scene: InteractiveScene,
    motion_loader: G1MotionLoader,
    coordinate_markers: VisualizationMarkers
):
    sim_dt = sim.get_physics_dt()
    
    frame_time = 0.0
    
    # Get body IDs for end effectors
    robot = scene["Robot"]
    

    while simulation_app.is_running():
        frame_time += sim_dt * args_cli.slow_down_factor
        
        # Get current frame data
        current_frame_data = motion_loader.get_frame_at_time(frame_time)
        joint_positions = motion_loader.get_joint_positions(current_frame_data)
        joint_velocities = motion_loader.get_joint_velocities(current_frame_data)
        end_effector_positions = motion_loader.get_end_effector_positions(current_frame_data)

        # Set fixed root state at 0.8m height (G1 default)
        root_state = scene["Robot"].data.default_root_state.clone()
        root_state[:, :3] = torch.tensor([0.0, 0.0, 0.8], device=scene.device) + scene.env_origins
        root_state[:, 3:7] = torch.tensor([1.0, 0.0, 0.0, 0.0], device=scene.device)  # Identity quaternion
        root_state[:, 7:] = 0.0  # Zero velocities

        # Map joint positions to robot joints
        robot_joint_positions = torch.zeros(NUM_ENVS, len(robot.data.joint_names), device=scene.device)
        robot_joint_velocities = torch.zeros(NUM_ENVS, len(robot.data.joint_names), device=scene.device)
        
        # Map motion data joint positions to robot joints using the mapping
        for motion_idx, robot_idx in motion_loader.joint_mapping.items():
            robot_joint_positions[:, robot_idx] = joint_positions[motion_idx]
            robot_joint_velocities[:, robot_idx] = joint_velocities[motion_idx]

        # Write states to simulation
        scene["Robot"].write_root_pose_to_sim(root_state[:, :7])
        scene["Robot"].write_root_velocity_to_sim(root_state[:, 7:])
        scene["Robot"].write_joint_state_to_sim(robot_joint_positions, robot_joint_velocities)
        
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)
        
        # After simulation step, get actual body positions and draw coordinate frames
        # Calculate world positions for end effectors relative to root
        root_pos = root_state[0, :3]  # Root position in world frame
        
        # Convert relative positions from motion data to world positions
        # Motion data format: left_hand(3), right_hand(3), left_foot(3), right_foot(3)
        end_effector_world_pos = torch.zeros(4, 3, device=scene.device)

        end_effector_world_pos[0] = root_pos + end_effector_positions[0]  # left_hand
        end_effector_world_pos[1] = root_pos + end_effector_positions[1]  # right_hand  
        end_effector_world_pos[2] = root_pos + end_effector_positions[2]  # left_foot
        end_effector_world_pos[3] = root_pos + end_effector_positions[3]  # right_foot

        # end_effector_world_pos[0] = end_effector_positions[0]  # left_hand
        # end_effector_world_pos[1] = end_effector_positions[1]  # right_hand  
        # end_effector_world_pos[2] = end_effector_positions[2]  # left_foot
        # end_effector_world_pos[3] = end_effector_positions[3]  # right_foot

        
        # Visualize end effector coordinate frames
        draw_end_effector_frames(
            scene,
            coordinate_markers,
            end_effector_world_pos,
            scene.env_origins
        )


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view((2.5, 0.0, 1.5), (0.0, 0.0, 0.8))

    # Prepare file path
    file_path = args_cli.file_path
    if not os.path.isabs(file_path):
        # Make relative to current script directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_dir, file_path)

    # Create motion loader and joint mapping
    motion_loader = G1MotionLoader(file_path, device=args_cli.device)
    print(f"Loaded motion: {motion_loader.num_frames} frames")

    # Create scene
    scene_cfg = G1SceneCfg(NUM_ENVS, env_spacing=10.0)
    scene = InteractiveScene(scene_cfg)
    coordinate_markers = create_coordinate_markers()

    sim.reset()
    print("[INFO] G1 joint position visualization setup complete. Simulating...")

    # Get robot data after reset and create joint mapping
    robot = scene["Robot"]
    motion_loader.create_joint_mapping(robot.data.joint_names)
    
    print("\nRobot joint names:")
    for idx, name in enumerate(robot.data.joint_names):
        print(f"  {idx}: {name}")
    print("\nMotion joint names:")
    for idx, name in enumerate(motion_loader.motion_joint_names):
        print(f"  {idx}: {name}")
    
    run_simulator(sim, scene, motion_loader, coordinate_markers)

if __name__ == "__main__":
    main()
    simulation_app.close()
