from grid import *
import copy
import math
from particle import Particle
from utils import *
from setting import *
import random

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    dx, dy, dh = odom
    motion_particles = []
    # Move the particles
    for (px, py, ph) in [p.xyh for p in particles]:
        #px, py, ph = add_odometry_noise(?,?,?)
        new_dx, new_dy = rotate_point(dx, dy, ph + dh)
        moved_particle = Particle(px + new_dx, py + new_dy, (ph + dh) % 360)
        motion_particles.append(moved_particle)

    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
                * Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """

    def get_marker_prob(p, o, grid):
        px, py, ph = p.xyh
        if not grid.is_free(px, py):
            return 0
        ox, oy, oh = o
        true_x, true_y = rotate_point(px, py, oh)
        dist_diff = grid_distance(ox, oy, px, py)
        angle_diff = diff_heading_deg(oh, ph)
        exp_formula = float(dist_diff ** 2) / (2 * (MARKER_TRANS_SIGMA ** 2)) + float(angle_diff ** 2) / (2 * (MARKER_ROT_SIGMA ** 2))
        marker_prob = math.exp(-1 * exp_formula)
        return marker_prob

    new_particles = []
    particle_probs = []

    for p in particles:
        particle_prob = 1
        for o in measured_marker_list:
            marker_prob = get_marker_prob(p, o, grid)
            particle_prob *= marker_prob
        particle_probs.append(particle_prob)

    print(particle_probs)
            
    return particles
