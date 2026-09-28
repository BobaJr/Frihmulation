from cmath import pi

from vpython import *
from numpy import *
import vpython as vp
import numpy as np

"""
HOW TO READ THIS CODE:
- All other variables are assumed to be in standard units
    -However, orientational initialization parameters(roll, pitch AOA) are in degrees
- Common abbreviations include:
    - AoA: Angle of Attack
    - Dir: Direction
    - Mag: Magnitude
    - Co: Coefficient
    - Const: Constant
    - Vec: Vector
- Vectors defined using VPython's vector class are used when matrix operations are not needed because they are easier to code with. All other vectors are defined using numpy arrays.
- All rotations follow the right-hand rule. (i.e. when looking along the rotation axis from the origin, rotations by positive angles should be clockwise, and vice versa.)
"""

#mutiplying quaternions
def multiply(x,y):
    a, b, c, d = x
    A, B, C, D = y
    return np.array([a*A - b*B - c*C - d*D, a*B + b*A + c*D - d*C, a*C - b*D + c*A + d*B, a*D + b*C - c*B + d*A])

def axisangle(axis, angle):
    return np.array([cos(angle/2), axis[0]*sin(angle/2), axis[1]*sin(angle/2), axis[2]*sin(angle/2)])

def rotatequat(v, q):
    conqugation=np.array([q[0], -q[1], -q[2], -q[3]])
    vectorquat=np.array([0, v[0], v[1], v[2]])
    return multiply(multiply(q, vectorquat), conqugation)[1:]

def updatequat(q, omega, t):
    q=q+0.5*t*multiply(q, np.array([0, omega[0], omega[1], omega[2]]))
    return q

def eulertoquat(roll, pitch, yaw):
    qroll = axisangle(np.array([1, 0, 0]), roll)
    qpitch = axisangle(np.array([0, 1, 0]), pitch)
    qyaw = axisangle(np.array([0, 0, 1]), yaw)
    return multiply(multiply(qyaw, qpitch), qroll)

class Frisbee(vp.cylinder):
    # REGISTRY OF ALL CLASS INSTANCES
    all_frisbees = []

    # CONSTRUCTOR
    def __init__(self, pos, vel, roll, pitch, omega): #INPUT: roll/pitch in deg, spin in rev/s --> translated to RADs

        # DIRECT INPUTS
        super().__init__(pos=pos, axis=vp.vec(0, 0.2, 0), color=vp.color.yellow, radius=1, make_trail = True)
        self.pos=pos
        self.vel=vel
        self.q = eulertoquat(roll*pi/180, pitch*pi/180, 0)
        self.q /=np.linalg.norm(self.q)
        # self.roll = roll * (pi/180)
        # self.pitch = pitch * (pi/180)
        # self.spin=spin

        # # OTHER ATTRIBUTES
        # # Reference axes
        # self.lift.dir = vec(0,0,0)
        # self.drag.dir = vec(0,0,0)
        # # Forces
        # self.planform_area = 0
        # self.speed = 0
        # self.air_const_forces = 0

        #self.co_lift = 0
        #self.co_drag = 0

        # self.lift_force_mag = 0
        # self.drag_force_mag = 0
        # self.gravity_force_mag = 0

        # self.lift_force = vec(0,0,0)
        # self.drag_force = vec(0,0,0)
        # self.gravity_force = vec(0,0,0)
        # # Angular orientations
        # self.AoA = 0
        self.orientation = np.array([roll*(pi/180), pitch*(pi/180), 0])

        self.prev_AoA = None
        self.prev_phi = 0
        self.prev_theta = 0
        self.prev_gamma = 0 # previous yaw angle for calculating gamma_prime

        # # Angular velocities
        spin = omega # currently passed as rev/s 

        self.omega = rotatequat(np.array([0.0, 2*np.pi*spin, 0.0]), self.q) 
        self.angular_momentum = 2.35e-3 * self.omega ### replace 2.35e-3 w/ I[2,2] and move consts to top?
        # self.angular_vel = np.array([0, 0, 0])
        # # Spin moments
        # self.spin_moment = np.array([0, 0, 0])
        # omega=2*np.pi*spin*np.array([0, 0, 1])

        # INITIALIZING CYLINDER OBJECT

        # APPENDING INSTANCE TO REGISTRY
        Frisbee.all_frisbees.append(self)

# CONSTANTS
g=9.81 # accel due to gravity
m = 0.175 # mass of frisbee
I = np.array([[1.22e-3, 0., 0.], 
              [0., 1.22e-3, 0.], 
              [0., 0., 2.35e-3]]) # inertia matrix of frisbee
RHO = 1.23 # air density
AREA = 0.0568 # planform area of horizontal frisbee
d = 2*sqrt(AREA/pi) # diameter of frisbee
h = 0.03175 # height of frisbee

# Flight coefficients from Hummel flight fshh3
CMA = -0.16
CMA_prime = 0.029
CRG_prime = 0.012 # C_r, gamma where gamma = yaw

#From some other paper? uses different set of coefficients than Hummel
CL0 = 0.1 # lift coefficient at zero angle of attack
CLA = 1.4 # lift coefficient slope
CD0 = 0.08 # drag coefficient at zero angle of attack
CDA = 2.72 # drag coefficient slope
ALPHA0 = -4 * (pi/180) # angle of attack that yields zero lift
# #spin coefficients
# CRR = 1.4e-2
# CRP = -5.5e-3
# CMO = -0.08
# CMA = 0.43
CMQ = -5e-3
CNR = -7.1e-6

# SIMULATION RATE
dt=0.001 # time step

# INITIALIZING OBJECTS
# Frisbee
frih = Frisbee(vp.vec(0, 1, 0), vp.vec(20, 0, 0), 12, 10, 20) # initial conditions
frih_bot = vp.cylinder(pos=vp.vec(0, 1, 0), axis=vp.vec(0, -0.02, 0), color=vp.color.black, radius=1, make_trail = False)
# Ground
platform = vp.box(pos=vp.vec(100, 0, 0), size=vp.vec(200, 0.1, 10), color=vp.color.white)
for i in range(10):
    vp.box(pos=vp.vec(10*i, 0.05, 0), size=vp.vec(1, 0.1, 10), color=vp.color.red)

# CAMERA SETTINGS
vp.scene.camera.follow(frih)
vp.scene.range = 15
vp.scene.forward = vp.vec(-1, -0.5, -1)

# START BUTTON
def run_animation(x):
    return x.checked
radio = vp.radio(bind=run_animation, text="RUN")



# MAIN LOOP
# Check if at least 1 frisbee is still flying
while(run_animation(radio) == False):
    pass
while(True):
    vp.rate(1/dt)
    for frisbee in Frisbee.all_frisbees:
        # FORCES
        # Magnitudes
        # lift_dir = vp.vec(0, 1, 0)
        # lift_dir = vp.rotate(lift_dir, angle=frisbee.roll, axis=vp.vec(frisbee.vel.x, 0, frisbee.vel.z))
        # lift_dir_rot_axis = vp.rotate(vp.vec(frisbee.vel.z, 0, -frisbee.vel.x), angle=frisbee.roll, axis=vp.vec(frisbee.vel.x, 0, frisbee.vel.z))
        # # ^^^ using the base rot axis vec(-frisbee.vel.z, 0, frisbee.vel.x) would result in positive values for pitch
        # # ^^^ Turning DOWN the nose/front of the frisbee
        # lift_dir = vp.rotate(lift_dir, angle=frisbee.pitch, axis=lift_dir_rot_axis)
        disc_normal = rotatequat(np.array([0.0, 1.0, 0.0]), frisbee.q)
        disc_normal /= np.linalg.norm(disc_normal)

        v = np.array([frisbee.vel.x, frisbee.vel.y, frisbee.vel.z]) ###change to vel_np for consistency?

        speed = np.linalg.norm(v)
        speedir = v / speed 
        vnormal = np.dot(speedir, disc_normal)

        vplane = speedir-vnormal*disc_normal
        vplane_mag = np.linalg.norm(vplane)

        d1=vplane/vplane_mag
        AoA = -np.arcsin(np.clip(np.dot(vnormal, disc_normal), -1.0, 1.0))

        # d1 = vp.cross(disc_normal, frisbee.vel).norm()
        # AoA = vp.diff_angle(d1, frisbee.vel)

        # planform_area = abs(AREA * cos(frisbee.roll) * cos(frisbee.pitch+AoA))
        speed = vp.mag(vp.vec(frisbee.vel.x, frisbee.vel.y, frisbee.vel.z))

        air_const_forces = 0.5 * RHO * AREA * speed**2

        co_lift = CL0+CLA* AoA
        co_drag = CD0+CDA * pow((AoA-ALPHA0), 2)

        lift_force_mag = co_lift * air_const_forces
        drag_force_mag = co_drag * air_const_forces
        gravity_force_mag = m * g
        # Normalized directions
        lift_dir_np = disc_normal - np.dot(disc_normal, vnormal)*vnormal
        lift_dir_np /= np.linalg.norm(lift_dir_np)

        drag_dir_np = -vnormal
        lift_dir = vp.vec(*lift_dir_np)
        drag_dir = vp.vec(*drag_dir_np) ###does vnormal represent direction of motion? why is it a float?

        # drag_dir = vp.vec(-frisbee.vel.x, -frisbee.vel.y, -frisbee.vel.z).norm()
        # Force vectors

        lift_force = lift_force_mag * lift_dir
        drag_force = drag_force_mag * drag_dir
        gravity_force = vp.vec(0, -gravity_force_mag, 0)

        net_force = lift_force + drag_force + gravity_force

        # POSITION UPDATE
        frisbee.vel += net_force/m * dt
        frisbee.pos += frisbee.vel * dt

        # ROLL MOMENTS
        # Initial orientation
        # frisbee.orientation = np.array([frisbee.roll, frisbee.pitch, 0])

        d3 = -disc_normal ###disc.normal?
        d2 = np.cross(d3, d1)
        d2 /= np.linalg.norm(d2)
        # d2 = vp.cross(d1, lift_dir).norm()
        # d3 = vp.cross(d1, d2).norm() # we are SHEPHERDS

        if frisbee.prev_AoA is None:
            AoA_prime = 0.0
        else:
            AoA_prime = (AoA - frisbee.prev_AoA)/dt

        # phi_prime = (frisbee.orientation[0] - frisbee.prev_phi)/dt
        # theta_prime = (frisbee.orientation[1] - frisbee.prev_theta)/dt
        # gamma_prime = (frisbee.orientation[2] - frisbee.prev_gamma)/dt
        # # omega = vp.vec(phi_prime*cos(frisbee.orientation[1]), theta_prime, phi_prime*sin(frisbee.orientation[1]+gamma_prime))

        omega = frisbee.omega
        omegad1 = np.dot(omega, d1)
        omegad2 = np.dot(omega, d2)
        omegad3 = np.dot(omega, d3)

        air_const_moments = air_const_forces*d

        M1=air_const_moments * (CMA*omegad1)
        M2 = air_const_moments*(CMA*AoA+CMQ*omegad2)
        M3 = CNR*omegad3
        torque_np = M1*d1+M2*d2+M3*d3

        #torque_np=np.array([torque.x, torque.y, torque.z])
        #Angular velocity

        frisbee.angular_momentum += torque_np * dt ###torque.np?
        lift_dir_np = np.array([lift_dir.x, lift_dir.y, lift_dir.z])

        am_parallel = np.dot(frisbee.angular_momentum, disc_normal)*disc_normal
        am_perpendicular = frisbee.angular_momentum-am_parallel
        omega= am_parallel/I[2,2]+am_perpendicular/I[0,0]
        frisbee.omega=omega
        frisbee.q = updatequat(frisbee.q, frisbee.omega, dt)

        frisbee.prev_AoA = AoA

        n=rotatequat(np.array([0., 1. , 0.]), frisbee.q)

        frisbee.axis = vp.vec(n[0], n[1], n[2]) * 0.2

    # CHECK FOR STOP CONDITION
    for frisbee in Frisbee.all_frisbees:
        if (
            speed== 0 
            and lift_force+drag_force+gravity_force == 0
            and vp.mag(frisbee.angular_vel) == 0 ###frisbee.omega?
            # and mag(spin_moment) == 0
        ):
            break

        
