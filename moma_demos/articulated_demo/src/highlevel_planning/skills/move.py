import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from highlevel_planning.tools.util import homogenous_trafo, invert_hom_trafo
from matplotlib import pyplot as plt

from highlevel_planning.tools.util import IKError

EPS = 1e-6
DEBUG = True


def ortho_projection(direction):
    assert np.abs(np.linalg.norm(direction) - 1.0) < EPS
    projection = np.matmul(direction, direction.T)
    return np.eye(3) - projection


def draw_arrow(vec_wristframe, robot, color, arrow_id=None, length=0.2):
    if DEBUG:
        link_state = p.getLinkState(
            robot.model.uid, robot.link_name_to_index["panda_default_EE"]
        )
        pos = np.array(link_state[4])
        orient = R.from_quat(link_state[5])
        vec_worldframe = orient.apply(np.squeeze(vec_wristframe))
        if arrow_id is None:
            return robot._world.draw_arrow(pos, vec_worldframe, color, length=length)
        else:
            return robot._world.draw_arrow(
                pos, vec_worldframe, color, length=length, replace_id=arrow_id
            )


class SkillMove:
    def __init__(self, scene_, robot_, desired_velocity, time_step):
        self.scene = scene_
        self.robot = robot_
        self.desired_velocity = desired_velocity

        self.gamma_direction = 2.0
        self.gamma_hinge = 0.2

        self.orthogonal_correction = False

        self.f_desired = np.zeros((3, 1))
        self.k_p_f = 0.8
        self.k_i_f = 0.0

        self.t_desired = np.zeros((3, 1))
        self.k_p_t = 0.0
        self.k_i_t = 0.0

        self.dt = time_step

        self.cart_vel_ctrl = None

    def move_object(self, desired_distance, direction_initial_guess):

        travelled_distance = 0.0
        last_position, _ = self.robot.get_link_pose("panda_default_EE")

        direction = np.copy(direction_initial_guess)
        direction /= np.linalg.norm(direction_initial_guess)
        direction = direction.reshape(3, 1)

        hinge_vector = np.zeros((3, 1))

        force_integral = np.zeros((3, 1))
        torque_integral = np.zeros((3, 1))

        arrow_1_id = draw_arrow(direction, self.robot, "green")
        arrow_2_id = draw_arrow(np.array([0.1, 0.0, 0.0]), self.robot, "red")
        if self.orthogonal_correction:
            arrow_3_id = draw_arrow(np.array([0.1, 0.0, 0.0]), self.robot, "yellow")

        plot_time = np.array([0.0])
        plot_force_data = np.zeros((3, 1))
        plot_direction_data = np.copy(direction)

        current_time = 0.0

        while travelled_distance < desired_distance:
            plot_time = np.append(plot_time, current_time)
            current_time += self.robot._world.T_s

            # Measure force and torque
            f_wristframe, t_wristframe = self.robot.get_wrist_force_torque()
            f_wristframe = f_wristframe.reshape(3, 1)
            t_wristframe = t_wristframe.reshape(3, 1)
            draw_arrow(
                f_wristframe,
                self.robot,
                "red",
                arrow_id=arrow_2_id,
                length=np.linalg.norm(f_wristframe) / 22.0,
            )
            plot_force_data = np.append(plot_force_data, f_wristframe, axis=1)

            # ---- Translation -----

            # Compute force reaction (PI controller)
            force_error = f_wristframe - self.f_desired
            projection_matrix = ortho_projection(direction)
            force_integral += self.dt * np.matmul(projection_matrix, force_error)
            v_f = self.k_p_f * force_error + self.k_i_f * force_integral
            if self.orthogonal_correction:
                draw_arrow(
                    np.matmul(projection_matrix, v_f),
                    self.robot,
                    "yellow",
                    arrow_id=arrow_3_id,
                )

            # Update direction estimate
            if self.orthogonal_correction:
                correction_vector = np.matmul(projection_matrix, v_f)
            else:
                correction_vector = v_f
            direction -= (
                self.dt
                * self.gamma_direction
                * self.desired_velocity
                * correction_vector
            )
            direction /= np.linalg.norm(direction)
            draw_arrow(direction, self.robot, "green", arrow_id=arrow_1_id)
            plot_direction_data = np.append(plot_direction_data, direction, axis=1)

            # Compute new translation velocity reference
            velocity_translation = self.desired_velocity * direction  # - np.matmul(
            #     projection_matrix, v_f
            # )

            # ---- Rotation -----

            # Torque reaction
            torque_error = t_wristframe - self.t_desired
            torque_integral += self.dt * torque_error
            w_t = self.k_i_t * torque_error + self.k_i_t * torque_integral

            # Update hinge estimate
            hinge_vector -= self.dt * self.gamma_hinge * self.desired_velocity * w_t

            # Compute new rotation velocity reference
            velocity_rotation = self.desired_velocity * hinge_vector - w_t
            
            # ---- Apply for one step --------

            self.robot.task_space_velocity_control(
                np.squeeze(velocity_translation), np.squeeze(velocity_rotation), 1
            )

            # Update travelled distance
            new_position, _ = self.robot.get_link_pose("panda_default_EE")
            travelled_distance += np.linalg.norm(new_position - last_position)
            last_position = new_position

        if DEBUG:
            # filename = "/home/fjulian/Desktop/drawer2.pdf"
            filename = None
            self.plot_data(plot_time, plot_direction_data, plot_force_data, filename)

        return True

    def plot_data(self, plot_time, plot_direction_data, plot_force_data, filename=None):
        print("Plotting data...")
        plot_force_color = self.robot._world.colors["red"]
        plot_dir_color = self.robot._world.colors["green"]
        _, axs_dir = plt.subplots(3, 1, figsize=(9, 7))
        axs_force = []
        plot_data = {"ylabel": ["x", "y", "z"]}
        for i in range(3):
            axs_dir[i].set_xlim(0.0, plot_time[-1])
            axs_dir[i].plot(
                plot_time,
                plot_direction_data[i, :],
                label="Direction",
                color=plot_dir_color,
            )
            axs_dir[i].set_ylabel(plot_data["ylabel"][i], color=plot_dir_color)
            axs_dir[i].tick_params(axis="y", labelcolor=plot_dir_color)

            axs_force.append(axs_dir[i].twinx())
            axs_force[i].plot(
                plot_time, plot_force_data[i, :], label="Force", color=plot_force_color
            )
            axs_force[i].set_ylabel(plot_data["ylabel"][i], color=plot_force_color)
            axs_force[i].tick_params(axis="y", labelcolor=plot_force_color)
        axs_dir[0].plot(
            plot_time,
            -0.7071 * np.ones(plot_time.shape),
            color=plot_dir_color,
            linestyle="--",
            label="Direction GT",
        )
        axs_dir[1].plot(
            plot_time,
            0.0 * np.ones(plot_time.shape),
            color=plot_dir_color,
            linestyle="--",
        )
        axs_dir[2].plot(
            plot_time,
            -0.7071 * np.ones(plot_time.shape),
            color=plot_dir_color,
            linestyle="--",
        )
        axs_dir[0].tick_params(labelbottom=False)
        axs_dir[1].tick_params(labelbottom=False)
        axs_dir[2].set_xlabel("Time [s]")
        axs_dir[0].legend(loc=2)
        axs_force[0].legend(loc=4)

        if filename is None:
            plt.show()
        else:
            plt.savefig(filename)
