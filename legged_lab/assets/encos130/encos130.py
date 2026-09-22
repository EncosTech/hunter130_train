"""Configuration for Encos130 humanoid robot (23 DOF, with waist and ankle roll joints)."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from legged_lab.assets import ISAAC_ASSET_DIR

ENCOS130_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=f"{ISAAC_ASSET_DIR}/encos130/urdf/encos130.urdf",
        # Reuse the converted USD across launches.
        usd_dir="/tmp/IsaacLab/encos130",
        usd_file_name="encos130.usd",
        fix_base=False,
        merge_fixed_joints=True,
        # Simplify the high-resolution CAD collision meshes for parallel simulation.
        collider_type="convex_hull",
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=None, damping=None)
        ),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # The crouched default pose places the sole about 0.65 m below the pelvis.
        pos=(0.0, 0.0, 0.7),
        joint_pos={
            # Left leg
            "left_hip_pitch_joint": -0.25,
            "left_hip_roll_joint": 0.00,
            "left_hip_yaw_joint": 0.00,
            "left_knee_joint": 0.50,
            "left_ankle_pitch_joint": -0.25,
            "left_ankle_roll_joint": 0.00,

            # Right leg
            "right_hip_pitch_joint": -0.25,
            "right_hip_roll_joint": 0.00,
            "right_hip_yaw_joint": 0.00,
            "right_knee_joint": 0.50,
            "right_ankle_pitch_joint": -0.25,
            "right_ankle_roll_joint": 0.00,

            # Waist
            "waist_yaw_joint": 0.00,
            "waist_roll_joint": 0.00,
            "waist_pitch_joint": 0.00,

            # Left arm
            "left_shoulder_pitch_joint": -0.10,
            "left_shoulder_roll_joint": 0.20,
            "left_shoulder_yaw_joint": 0.00,
            "left_elbow_joint": -0.50,

            # Right arm
            "right_shoulder_pitch_joint": -0.10,
            "right_shoulder_roll_joint": -0.20,
            "right_shoulder_yaw_joint": 0.00,
            "right_elbow_joint": -0.50,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_pitch_joint",
                ".*_hip_roll_joint",
                ".*_hip_yaw_joint",
                ".*_knee_joint",
            ],
            effort_limit_sim={
                ".*_hip_pitch_joint": 150.0,
                ".*_hip_roll_joint": 150.0,
                ".*_hip_yaw_joint": 150.0,
                ".*_knee_joint": 150.0,
            },
            velocity_limit_sim={
                ".*_hip_pitch_joint": 25.0,
                ".*_hip_roll_joint": 25.0,
                ".*_hip_yaw_joint": 25.0,
                ".*_knee_joint": 25.0,
            },
            stiffness={
                ".*_hip_pitch_joint": 300.0,
                ".*_hip_roll_joint": 300.0,
                ".*_hip_yaw_joint": 200.0,
                ".*_knee_joint": 300.0,
            },
            damping={
                ".*_hip_pitch_joint": 15.0,
                ".*_hip_roll_joint": 15.0,
                ".*_hip_yaw_joint": 10.0,
                ".*_knee_joint": 15.0,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_ankle_pitch_joint",
                ".*_ankle_roll_joint",
            ],
            effort_limit_sim={
                ".*_ankle_pitch_joint": 80.0,
                ".*_ankle_roll_joint": 38.0,
            },
            velocity_limit_sim={
                ".*_ankle_pitch_joint": 25.0,
                ".*_ankle_roll_joint": 25.0,
            },
            stiffness={
                ".*_ankle_pitch_joint": 100.0,
                ".*_ankle_roll_joint": 100.0,
            },
            damping={
                ".*_ankle_pitch_joint": 5.0,
                ".*_ankle_roll_joint": 5.0,
            },
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=[
                "waist_yaw_joint",
                "waist_roll_joint",
                "waist_pitch_joint",
            ],
            effort_limit_sim={
                "waist_yaw_joint": 150.0,
                "waist_roll_joint": 140.0,
                "waist_pitch_joint": 112.5,
            },
            velocity_limit_sim={
                "waist_yaw_joint": 25.0,
                "waist_roll_joint": 25.0,
                "waist_pitch_joint": 25.0,
            },
            stiffness={
                "waist_yaw_joint": 300.0,
                "waist_roll_joint": 300.0,
                "waist_pitch_joint": 300.0,
            },
            damping={
                "waist_yaw_joint": 5.0,
                "waist_roll_joint": 5.0,
                "waist_pitch_joint": 5.0,
            },
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_pitch_joint",
                ".*_shoulder_roll_joint",
                ".*_shoulder_yaw_joint",
                ".*_elbow_joint",
            ],
            effort_limit_sim={
                ".*_shoulder_pitch_joint": 30.0,
                ".*_shoulder_roll_joint": 30.0,
                ".*_shoulder_yaw_joint": 30.0,
                ".*_elbow_joint": 30.0,
            },
            velocity_limit_sim={
                ".*_shoulder_pitch_joint": 25.0,
                ".*_shoulder_roll_joint": 25.0,
                ".*_shoulder_yaw_joint": 25.0,
                ".*_elbow_joint": 25.0,
            },
            stiffness={
                ".*_shoulder_pitch_joint": 50.0,
                ".*_shoulder_roll_joint": 50.0,
                ".*_shoulder_yaw_joint": 50.0,
                ".*_elbow_joint": 50.0,
            },
            damping={
                ".*_shoulder_pitch_joint": 1.5,
                ".*_shoulder_roll_joint": 1.5,
                ".*_shoulder_yaw_joint": 1.5,
                ".*_elbow_joint": 1.5,
            },
        ),
    },
)
