"""Velocity commands expressed in the torso yaw frame."""

import torch
import isaaclab.utils.math as math_utils
from isaaclab.envs.mdp.commands import UniformVelocityCommand


class TorsoVelocityCommand(UniformVelocityCommand):
    def __init__(self, cfg, env):
        self.torso_body_id = env.torso_body_id
        super().__init__(cfg, env)

    def _torso_yaw(self):
        return math_utils.yaw_quat(self.robot.data.body_quat_w[:, self.torso_body_id])

    def _linear_velocity(self):
        return math_utils.quat_apply_inverse(self._torso_yaw(), self.robot.data.root_lin_vel_w)

    def _update_command(self):
        if self.cfg.heading_command:
            ids = self.is_heading_env.nonzero(as_tuple=False).flatten()
            forward = math_utils.quat_apply(
                self.robot.data.body_quat_w[:, self.torso_body_id], self.robot.data.FORWARD_VEC_B
            )
            heading = torch.atan2(forward[:, 1], forward[:, 0])
            error = math_utils.wrap_to_pi(self.heading_target[ids] - heading[ids])
            self.vel_command_b[ids, 2] = torch.clamp(
                self.cfg.heading_control_stiffness * error, *self.cfg.ranges.ang_vel_z
            )
        self.vel_command_b[self.is_standing_env] = 0.0

    def _update_metrics(self):
        steps = self.cfg.resampling_time_range[1] / self._env.step_dt
        self.metrics["error_vel_xy"] += torch.linalg.vector_norm(
            self.command[:, :2] - self._linear_velocity()[:, :2], dim=1
        ) / steps
        self.metrics["error_vel_yaw"] += (
            self.command[:, 2] - self.robot.data.body_ang_vel_w[:, self.torso_body_id, 2]
        ).abs() / steps

    def _resolve_xy_velocity_to_arrow(self, xy_velocity):
        scale = torch.tensor(self.cfg.goal_vel_visualizer_cfg.markers["arrow"].scale, device=self.device)
        scale = scale.repeat(xy_velocity.shape[0], 1)
        scale[:, 0] *= torch.linalg.vector_norm(xy_velocity, dim=1) * 3.0
        angle = torch.atan2(xy_velocity[:, 1], xy_velocity[:, 0])
        zeros = torch.zeros_like(angle)
        local_quat = math_utils.quat_from_euler_xyz(zeros, zeros, angle)
        return scale, math_utils.quat_mul(self._torso_yaw(), local_quat)

    def _debug_vis_callback(self, event):
        if not self.robot.is_initialized:
            return
        position = self.robot.data.body_pos_w[:, self.torso_body_id].clone()
        position[:, 2] += 0.3
        goal_scale, goal_quat = self._resolve_xy_velocity_to_arrow(self.command[:, :2])
        actual_scale, actual_quat = self._resolve_xy_velocity_to_arrow(self._linear_velocity()[:, :2])
        self.goal_vel_visualizer.visualize(position, goal_quat, goal_scale)
        self.current_vel_visualizer.visualize(position, actual_quat, actual_scale)
