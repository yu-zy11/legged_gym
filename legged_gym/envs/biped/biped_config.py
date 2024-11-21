# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO


class BipedRoughCfg(LeggedRobotCfg):
    class env(LeggedRobotCfg.env):
        num_envs = 4096
        num_observations = 169
        num_actions = 12

    class terrain(LeggedRobotCfg.terrain):
        # 1mx1m rectangle (without center line)
        measured_points_x = [-0.5, -0.4, -0.3, -
                             0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        measured_points_y = [-0.5, -0.4, -0.3, -
                             0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        mesh_type = 'plane'

    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.836]  # x,y,z [m]
        default_joint_angles = {  # = target angles [rad] when action = 0.0
            'J01_HIP_ROLL_L': 0.0,
            'J02_HIP_YAW_L': 0.,
            'J03_HIP_PITCH_L': -0.12,
            'J04_KNEE_PITCH_L': 0.24,
            'J05_ANKLE_PITCH_L': -0.12,
            'J06_ANKLE_ROLL_L': 0,

            'J07_HIP_ROLL_R': 0.0,
            'J08_HIP_YAW_R': 0.,
            'J09_HIP_PITCH_R': -0.12,
            'J10_KNEE_PITCH_R': 0.24,
            'J11_ANKLE_PITCH_R': -0.12,
            'J12_ANKLE_ROLL_R': 0.
        }

    class commands:
        curriculum = False
        max_curriculum = 1.
        # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        num_commands = 4
        resampling_time = 10.  # time before command are changed[s]
        heading_command = False  # if true: compute ang vel command from heading error

        class ranges:
            lin_vel_x = [-1.0, 1.0]  # min max [m/s]
            lin_vel_y = [-0.5, 0.5]   # min max [m/s]
            ang_vel_yaw = [-1, 1]    # min max [rad/s]
            heading = [-3.14, 3.14]

    class control(LeggedRobotCfg.control):
        control_type = 'P'  # P: position, V: velocity, T: torques
        # PD Drive parameters:
        stiffness = {'HIP_ROLL': 160.0, 'HIP_YAW': 160.0,
                     'HIP_PITCH': 160., 'KNEE_PITCH': 160., 'ANKLE_PITCH': 20.,
                     'ANKLE_ROLL': 20.}  # [N*m/rad]
        damping = {'HIP_ROLL': 6.0, 'HIP_YAW': 6.0,
                   'HIP_PITCH': 6., 'KNEE_PITCH': 6., 'ANKLE_PITCH': 1.,
                   'ANKLE_ROLL': 1.}  # [N*m*s/rad]     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset(LeggedRobotCfg.asset):
        # file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/biped/xml/biped.xml'
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/biped/urdf/sa01_rl.urdf'
        name = "biped"
        foot_name = 'LINK_ANKLE_ROLL'
        terminate_after_contacts_on = ['LINK_BASE']
        flip_visual_attachments = False
        self_collisions = 0  # 1 to disable, 0 to enable...bitwise filter
        # fix_base_link = False  # fixe the base of the robot

        # penalize_contacts_on = []
        # terminate_after_contacts_on = []
        # disable_gravity = False

        # # see GymDofDriveModeFlags (0 is none, 1 is pos tgt, 2 is vel tgt, 3 effort)
        # default_dof_drive_mode = 3
        # # replace collision cylinders with capsules, leads to faster/more stable simulation
        # replace_cylinder_with_capsule = True
        # armature = 0.

    class rewards(LeggedRobotCfg.rewards):
        soft_dof_pos_limit = 0.95
        soft_dof_vel_limit = 0.8
        soft_torque_limit = 0.8
        max_contact_force = 350
        only_positive_rewards = False
        base_height_target = 0.83

        class scales:
            termination = -200.
            tracking_lin_vel = 1.0
            tracking_ang_vel = 0.5

            lin_vel_z = -0.5
            base_height = -0.5
            ang_vel_xy = -0.1
            orientation = -1.0

            feet_air_time = 5.
            no_fly = 0.25
            feet_contact_forces = -0.
            collision = -1.

            dof_pos_limits = -0

            action_rate = -0.05
            torques = -1.e-5
            dof_vel = -0.0
            dof_acc = -8.e-8

            feet_stumble = -0.0
            stand_still = -0.


class BipedRoughCfgPPO(LeggedRobotCfgPPO):

    class runner(LeggedRobotCfgPPO.runner):
        run_name = ''
        experiment_name = 'rough_biped'

    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01
