import numpy as np
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch, FancyArrowPatch
import mpl_toolkits.mplot3d.art3d as art3d
import mpl_toolkits.mplot3d as mp3d
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from progress.bar import *


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


def solve_ik(eva, guess, theta, xyz_absolute):
    with eva.lock():
        # Inputs: Theta [deg], xyz_absolute [m]
        pos = [xyz_absolute[0], xyz_absolute[1], xyz_absolute[2]]  # [m]
        pos_json = {'x': (pos[0]), 'y': (pos[1]), 'z': (pos[2])}
        # This provides a rotation of theta [deg] wrt the end effector orientation pointing downwards
        orient_rel = [np.cos(np.deg2rad(theta)/2), 0, 0, np.sin(np.deg2rad(theta)/2)]
        orient_abs = quaternion_multiply([0, 0, 1, 0], orient_rel)
        orient_json = {'w': (orient_abs[0]), 'x': (orient_abs[1]), 'y': (orient_abs[2]), 'z': (orient_abs[3])}
        # Compute IK
        result_ik = eva.calc_inverse_kinematics(guess, pos_json, orient_json)
        success_ik = result_ik['ik']['result']
        joints_ik = result_ik['ik']['joints']
    return success_ik, joints_ik


class EvaGrids:
    def __init__(self, eva, config, show_plot):
        self.config = config
        self.eva = eva
        self.show_plot = show_plot
        self.counter_max = {self.config['grids']['names'][0]: self.config['grids']['row'][self.config['grids']['names'][0]]*self.config['grids']['col'][self.config['grids']['names'][0]]-1,
                            self.config['grids']['names'][1]: self.config['grids']['row'][self.config['grids']['names'][1]]*self.config['grids']['col'][self.config['grids']['names'][1]]-1}
        # Check that slots in second grid are >= than first grid
        slots = [abs(self.config['grids']['row'][self.config['grids']['names'][0]]) * abs(self.config['grids']['col'][self.config['grids']['names'][0]]),
                 abs(self.config['grids']['row'][self.config['grids']['names'][1]]) * abs(self.config['grids']['col'][self.config['grids']['names'][1]])]
        if slots[0] > slots[1]:
            raise Exception('Drop-off grid is smaller than pick-up grid')

    def _create_grid(self, grid_iter):
        """Creates 2D grid from YAML configuration file
        grid_x, grid_y: list of lists containing all grid points, in cartesian space.
         Units: [m]"""
        row = abs(self.config['grids']['row'][grid_iter])
        col = abs(self.config['grids']['col'][grid_iter])
        row_pitch = abs(self.config['grids']['row_pitch'][grid_iter])
        col_pitch = abs(self.config['grids']['col_pitch'][grid_iter])
        x0 = self.config['grids']['x0'][grid_iter]
        y0 = self.config['grids']['y0'][grid_iter]
        surface = self.config['grids']['surface'][grid_iter]
        theta = np.deg2rad(self.config['grids']['angle'][grid_iter])
        grid_x = [[0 for _i in range(col)] for _j in range(row)]
        grid_y = [[0 for _i in range(col)] for _j in range(row)]
        grid_z = [[0 for _i in range(col)] for _j in range(row)]
        grid_points = []
        for i in range(row):
            for j in range(col):
                grid_x[i][j] = (np.cos(theta+np.pi/2)*col_pitch*j+np.sin(theta+np.pi/2)*row_pitch*i)+x0
                grid_y[i][j] = -(-np.sin(theta+np.pi/2)*col_pitch*j+np.cos(theta+np.pi/2)*row_pitch*i)+y0
                grid_z[i][j] = surface
        for i in range(row):
            for j in range(col):
                grid_points.append([grid_x[i][j], grid_y[i][j], grid_z[i][j]])

        return grid_x, grid_y, grid_z, grid_points

    def get_grid_points(self, grids):
        """Creates 2D grid using the _create_grid() method and extract the grid point
        corresponding to the object_name and counter selected. It then solves the IK
        for the pickup point [x, y, z] and the hover point [x, y, z + z_hover] and
        provides the corresponding joint angles"""
        if self.show_plot:
            fig = plt.figure(figsize=(10, 5))
            plt.style.use('seaborn-pastel')
            ax = [fig.add_subplot(121), fig.add_subplot(122, projection='3d')]
        joints = {grids[0]: {'pick': [], 'hover': []}, grids[1]: {'pick': [], 'hover': []}}

        for grid_iter in grids:
            obj_h = abs(self.config['grids']['object'][grid_iter])
            ee_h = abs(self.config['EVA']['end_effector']['length'])
            hover_h = abs(self.config['EVA']['hover_height'])
            guess = self.config['grids']['guess'][grid_iter]
            grid_x, grid_y, grid_z, grid_points = self._create_grid(grid_iter)

            with ChargingBar('Computing ' + self.config['grids']['names_verbose'][grid_iter] + ' grid',
                             max=len(grid_points)) as bar:
                for _counter in range(len(grid_points)):
                    x = grid_points[_counter][0]/1000  # [m]
                    y = grid_points[_counter][1]/1000  # [m]
                    z = (grid_points[_counter][2]+ee_h+obj_h)/1000  # [m]
                    pos_obj = [x, y, z]  # [m]
                    pos_obj_hover = [x, y, z + hover_h]  # [m]
                    extra_angle = self.config['grids']['angle_pickup'][grid_iter]     # Additional pickup angle
                    success_pick, joints_pick = solve_ik(self.eva, guess, extra_angle, pos_obj)
                    success_hover, joints_hover = solve_ik(self.eva, guess, extra_angle, pos_obj_hover)
                    if ('success' not in success_pick) or ('success' not in success_hover):
                        raise Exception('IK error')  # Failed IK. Position not reachable
                    joints[grid_iter]['pick'].append(joints_pick)
                    joints[grid_iter]['hover'].append(joints_hover)
                    bar.next()
                if self.show_plot:
                    self.plot_grids(grid_x, grid_y, grid_z, ax, grid_iter)
        if self.show_plot:
            plt.show()
        move_eva = input("Please verify the correctness of grid placement before continuing "
                         "(to do this, set plot_on_off variable to True and verify grids placement from the plots).\n"
                         "Proceed (this will move the robot)? yes/no\n")
        if 'yes' not in move_eva:
            raise Exception('\n\n\nScript aborted by user: \n - set the plot_on_off variable to True\n '
                            '- run the script again\n - verify grids placement from the plots ')
        return joints

    def plot_grids(self, grid_x, grid_y, grid_z, ax_all, grid_iter):
        all_points = []
        ax1 = ax_all[0]
        ax2 = ax_all[1]

        # Plot settings
        grid_color = {'A': 'blue', 'B': 'red'}
        box_side = 600
        arr_length = 300
        work = Circle((0, 0), self.config['EVA']['work_radius'] * 1000, color='k', alpha=0.1)
        base = self.config['EVA']['base_plate'] * 1000 / 2

        # 2D Plot
        base_plate_2d = FancyBboxPatch((-base, -base), base * 2, base * 2, edgecolor='black', facecolor='gray',
                                       alpha=0.2)
        io_port_2d = Rectangle((-0.08 * 1000, -0.08 * 1000 / 2), 0.015 * 1000, 0.08 * 1000, edgecolor='green',
                               facecolor='green')
        grid_vertices_2d = [(grid_x[0][0], grid_y[0][0]), (grid_x[-1][0], grid_y[-1][0]),
                            (grid_x[-1][-1], grid_y[-1][-1]), (grid_x[0][-1], grid_y[0][-1])]
        grid_footprint_2d = mp3d.art3d.PolyCollection([grid_vertices_2d], color=grid_color[grid_iter], alpha=0.3)
        ax1.add_artist(plt.Circle((0, 0), self.config['EVA']['work_radius'] * 1000, facecolor='gray', alpha=0.1))
        ax1.add_patch(base_plate_2d), ax1.add_patch(io_port_2d)
        ax1.set_xlim(-box_side, box_side), ax1.set_ylim(-box_side, box_side)
        ax1.set_aspect('equal'), ax1.set_xlabel('x [mm]'), ax1.set_ylabel('y [mm]')
        ax1.arrow(0, 0, 200, 0, head_width=10, head_length=10, fc='red', ec='red', linewidth=2, alpha=0.6)
        ax1.arrow(0, 0, 0, 200, head_width=10, head_length=10, fc='green', ec='green', linewidth=2, alpha=0.6)
        ax1.plot(0, 0, 'o', markersize=5, color='blue', alpha=0.6)
        ax1.plot(grid_x, grid_y, 'o', color='grey', markersize=2)
        ax1.plot(grid_x[0][0], grid_y[0][0], '*', markersize=10, color='green')  # First point
        ax1.plot(grid_x[-1][-1], grid_y[-1][-1], 'X', markersize=8, color='red')  # Last point
        ax1.add_collection(grid_footprint_2d)
        blue_patch = mpatches.Patch(color='blue', label='Origin grid', alpha=0.4)
        red_patch = mpatches.Patch(color='red', label='Target grid', alpha=0.4)
        plt.legend(handles=[blue_patch, red_patch])

        for i in range(self.config['grids']['row'][grid_iter]):
            for j in range(self.config['grids']['col'][grid_iter]):
                all_points.append([grid_x[i][j], grid_y[i][j]])

        for i in range(len(all_points) - 1):
            ax1.arrow(all_points[i][0], all_points[i][1], (all_points[i + 1][0] - all_points[i][0]),
                      (all_points[i + 1][1] - all_points[i][1]), head_width=15, head_length=15,
                      fc='black', ec='black', linewidth=1, alpha=0.5)

        # 3D Plot
        base_plate_3d = [(-base, -base, 0), (base, -base, 0), (base, base, 0), (-base, base, 0)]
        base_plate_shape_3d = mp3d.art3d.Poly3DCollection([base_plate_3d], color='black', alpha=0.5, linewidth=1)
        grid_vertices_3d = [(grid_x[0][0], grid_y[0][0], grid_z[0][0]), (grid_x[-1][0], grid_y[-1][0], grid_z[0][0]),
                            (grid_x[-1][-1], grid_y[-1][-1], grid_z[0][0]),
                            (grid_x[0][-1], grid_y[0][-1], grid_z[0][0])]
        grid_footprint_3d = mp3d.art3d.Poly3DCollection([grid_vertices_3d], color=grid_color[grid_iter], alpha=0.3)

        a = Arrow3D([0, arr_length], [0, 0], [0, 0], mutation_scale=5, lw=1, arrowstyle="-|>", color="r")
        b = Arrow3D([0, 0], [0, arr_length], [0, 0], mutation_scale=5, lw=1, arrowstyle="-|>", color="g")
        c = Arrow3D([0, 0], [0, 0], [0, 0.5 * arr_length], mutation_scale=5, lw=1, arrowstyle="-|>", color="b")

        ax2.set_facecolor('white')
        ax2.add_collection3d(base_plate_shape_3d)
        ax2.add_patch(work)
        art3d.pathpatch_2d_to_3d(work, z=0, zdir="z")
        ax2.add_artist(a), ax2.add_artist(b), ax2.add_artist(c)
        ax2.xaxis.pane.set_edgecolor('black'), ax2.yaxis.pane.set_edgecolor('black'), ax2.zaxis.pane.set_edgecolor(
            'black')
        ax2.set_xlabel('x [mm]'), ax2.set_ylabel('y [mm]'), ax2.set_zlabel('z [mm]')
        ax2.set_xlim(-box_side, box_side), ax2.set_ylim(-box_side, box_side), ax2.set_zlim(0, 0.5 * box_side)
        ax2.add_collection3d(grid_footprint_3d)
        ax2.scatter(grid_x, grid_y, grid_z, 'o', s=2, color='black')
        ax2.scatter(grid_x[0][0], grid_y[0][0], grid_z[0][0], '*', s=10, color='green')  # First point
