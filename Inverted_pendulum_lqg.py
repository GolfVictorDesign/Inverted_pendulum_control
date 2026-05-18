"""
Created on 2022-06-18
@author: GolfVictorDesign

Simulation of a LQG control for inverted pendulum on wheels
"""
import numpy
import scipy.integrate as integrate
import control
import matplotlib.pyplot as plot
import matplotlib.animation as animation

from functools import partial

def animation_function(frame, artists:tuple):
    wheel, body = artists
    # xc, yc = wheel.center
    # vertices = body.get_xy()
    # xc = 0.1 * frame
    # wheel.centre = (xc, yc)
    print("update called\n")
    return (wheel, body)

class inverted_pendulum_robot:
    
    # Physical constant parameters
    _m_body_mass_m      = 0.20      # m, mass of the pendulum
    _m_cart_mass_M      = 0.30      # M, mass of the cart
    _m_wheel_mass_Mw    = 0.05      # Mw  mass of the wheel
    _m_total_mass_Mt    = _m_body_mass_m + _m_cart_mass_M + _m_wheel_mass_Mw   # Mt total mass
    _m_arm_length_l     = 0.15      # l, arm length from the cart to the center of mass of the pendulum
    _m_g                = 9.81      # gravity constant    _m_cart_inertia_Ic  = 0.0       # Inertia of the cart = Mx"
    _m_motor_Km         = 200 / (60 * 2 * numpy.pi * 12)     # Motor constant Km (r/V.sec) 200rpm@12v
    _m_motor_Ri         = 0.1       # motor inductor resistance    
    _m_arm_inertia_Ip   = (1/3) * _m_body_mass_m * (_m_arm_length_l ** 2)  # Ip inertia of an arm around it extremity
    _m_cart_inertia_Ic  = 0.0       # cart inertia = Mx"
    _m_motor_inertia_Im = 0.1       # Motor inertia
    _m_wheel_Rwe        = 0.05      # Wheel external radius Rwe (m)
    _m_wheel_Rwi        = 0.04      # Wheel internal radius Rwi (m)
    _m_wheel_Iw         = 0.5 * _m_wheel_mass_Mw * (_m_wheel_Rwe**2 + _m_wheel_Rwi**2)
    _m_J_coef           = 0.1
    _m_motor_Tm         = 0.0       # motor torque
    _m_motor_Bm         = 0.0       # motor viscous damping coef
    _m_cart_Bc          = 0.0       # cart viscous damping coef
    _m_combined_B       = 0.0       # combined linear damping coefficients
    _m_combined_Brot    = 0.0       # combined rotational damping coefficients ≈ 0
    _m_viscous_Ff       = 0.0       # viscous linear damping forces = Bx'
    _m_determinant      = 1.0       # determinant function of angle
    _m_u_input          = 0.0       # Input in V
    
    # System representation
    _m_y_labels_ode     = (("Position x","m"), ("Speed x'","m/s"), ("Position Theta","rad"), ("Speed Theta'","rad/s"))
    _m_integration_time = 0
    #                                   [[x1,  y1 ], [x2,   y2 ], [x3,  y3  ], [x4,   y4  ]]
    _m_body             = numpy.array(  [[-0.025, 0.0], [-0.025, 0.20], [0.025, 0.20], [0.025, 0.0]]) 
    _m_dt_sampling      = 0.0           # System loop sampling/integration time steps
    
    _m_continuous_system: control.NonlinearIOSystem
    
    def __init__(
        self,
        x_init = 0.0, 
        x_dot_init = 0.0, 
        theta_init = 0.0, 
        theta_dot_init = 0.0,
        motor_Bm = 0.0,
        dt_system = 0.001,
        u_input = 0.0,
        integration_time = 10) -> None:
        """ 
            Initialize and integrate the system model
        """
        # Setup of initial values
        self._m_motor_Bm = motor_Bm
        self._m_u_input = u_input  
        self._m_states_init = numpy.array([x_init, x_dot_init, theta_init, theta_dot_init])
        self._m_dt_sampling = dt_system 
        self._m_integration_time = integration_time
        
        self._solver = integrate.solve_ivp(
                                        self.__inverted_pendulum_model, 
                                        (0, self._m_integration_time), 
                                        self._m_states_init, 
                                        dense_output=True, 
                                        rtol=1e-5, 
                                        atol=1e-6 )
        
        # Setup solutions
        self._t_vis = numpy.arange(0, self._m_integration_time, self._m_dt_sampling)
        self._y_vis = self._solver.sol(self._t_vis)
         
    def __inverted_pendulum_model(self, t, y):
        
        x           = y[0]
        x_dot       = y[1]
        theta       = y[2]
        theta_dot   = y[3]
        
        """
            Tm = (Km / (Rwe.Ra)).u - (Km² / (Rwe²-Ra)).x'
                | Torque generated | counter torque generated |
                |    by current    |       By back EMF        |      
        """
        self._m_motor_Tm    =   (self._m_motor_Km / (self._m_wheel_Rwe * self._m_motor_Ri)) * self._m_u_input - \
                                    (self._m_motor_Km**2 / (self._m_wheel_Rwe**2 - self._m_motor_Ri)) * x_dot
                                    
        """
            B =     Km²/JRa     +     Bm/J      +   (Bc/J).Rwe²
                |  Back EMF     |     motor     |    rotational     |
                |  damping      |    damping    |   cart damping    |
            Ff = B.x'
        """      
        self._m_combined_B  =   (self._m_motor_Km**2 / (self._m_J_coef * self._m_motor_Ri)) + \
                                    (self._m_motor_Bm / self._m_J_coef) + ((self._m_cart_Bc / self._m_J_coef) * \
                                        self._m_wheel_Rwe**2)
        self._m_viscous_Ff  =   self._m_combined_B * x_dot 
        
        """
            D =     J.(Ip + ml²)    -   (m.l.rwe.cos(theta))²
                |   rotational      |   cinematic coupling      | 
                   System inertia   |       cart/body           |
        """
        self._m_determinant =   self._m_J_coef * \
                                    (self._m_arm_inertia_Ip + (self._m_body_mass_m * self._m_arm_length_l**2)) - \
                                        (self._m_body_mass_m * self._m_arm_length_l * self._m_wheel_Rwe * \
                                            numpy.cos(theta))**2
        
        """
            x" = ((Ip + ml²). \
                (Tm - Ff + m.l. \
                    theta'².sin(theta)) - (m.l. \
                        Rwe.cos(theta)).(m.g. \
                            l.sin(theta) - Brot.theta')) / \
                                D
        """
        x_ddot      = ((self._m_arm_inertia_Ip + self._m_body_mass_m * self._m_arm_length_l**2) * \
                        (self._m_motor_Tm - self._m_viscous_Ff + self._m_body_mass_m * self._m_arm_length_l * \
                            theta_dot**2 * numpy.sin(theta)) - (self._m_body_mass_m * self._m_arm_length_l * \
                                self._m_wheel_Rwe * numpy.cos(theta)) * ((self._m_body_mass_m * self._m_g * \
                                    self._m_arm_length_l * numpy.sin(theta)) - (self._m_combined_Brot * theta_dot))) / \
                                        self._m_determinant
        
        """
            Theta" = (-(m.l.Rwe.cos(theta)). \
                        (Tm - Ff + m.l. \
                            theta'².sin(theta)) + J.(m.g. \
                                l.sin(theta) - Brot.theta')) / \
                                    D
        """
        theta_ddot  = (-(self._m_body_mass_m * self._m_arm_length_l * self._m_wheel_Rwe * numpy.cos(theta)) * \
                        (self._m_motor_Tm - self._m_viscous_Ff + (self._m_body_mass_m * self._m_arm_length_l * \
                            theta_dot**2 * numpy.sin(theta))) + self._m_J_coef * ((self._m_body_mass_m * self._m_g * \
                                self._m_arm_length_l * numpy.sin(theta)) - (self._m_combined_Brot * theta_dot))) / \
                                    self._m_determinant                                    
                                        
        # Cart linear inertia
        self._m_cart_inertia_Ic = abs(self._m_cart_mass_M * x_ddot)
        
        # J = Ic + Im + Iw + MRwe²
        self._m_J_coef = self._m_cart_inertia_Ic + self._m_motor_inertia_Im + (self._m_wheel_mass_Mw * x_ddot) + \
                            (self._m_cart_mass_M * self._m_wheel_Rwe**2)
                
        return [x_dot, x_ddot, theta_dot, theta_ddot]

    def plot_dynamics(self) -> None:
        
        for index, state in enumerate(self._y_vis):
            _, ode_plot_axes = plot.subplots()
            ode_plot_axes.grid(True, 'major', linestyle = "-")
            ode_plot_axes.set(xlabel="time", ylabel=self._m_y_labels_ode[index][0])
            ode_plot_axes.legend(fontsize=14, labels=self._m_y_labels_ode[index][1])
            ode_plot_axes.plot(self._t_vis, state)
        
        plot.show(block=True)
            
    def animate_system(self) -> None:

        def init_animation(artists:tuple):
            wheel, body = artists
            animation_axes.add_patch(wheel)
            animation_axes.add_patch(body)
            return (wheel, body)
                
        wheel = plot.Circle((self._m_states_init[0], 0), self._m_wheel_Rwe*10, fc="b")
        body  = plot.Polygon(self._m_body*10, fc="y")
            
        animation_fig = plot.figure(figsize=(10,10))
        animation_axes = animation_fig.add_subplot(autoscale_on=False, xlim=(-5, 5), ylim=(-1, 9))
        animation_axes.grid(True, 'major', linestyle = "-")
        animation_axes.set(xlabel="X position")
        animation_axes.legend(fontsize=14)
        animation_axes.plot([0, 0], [1,1], lw=2)
        
        animation.FuncAnimation(
                        animation_fig, 
                        func=lambda frame : animation_function(frame, (wheel, body)),
                        frames=360, #len(self._t_vis), 
                        init_func=partial(init_animation, (wheel, body)),
                        interval=20, # self._m_dt_sampling*1000, 
                        blit=True )
        
        plot.title("Inverted pendulum robot animation")
        plot.show()

    def state_space_system(self):
        """
            A is the Jacobian of X :

                A11 = dX1/dX1 = 0   A12 = dX1/dX2 = 1    A13 = dX1/dX3 = 0   A14 = dX1/dX4 = 0
                A21 = dX2/dX1 = 0   A22 = dX2/dX2 = beta A23 = dX2/dX3 = 0   A24 = dX1/dX4 = 0
                A31 = dX3/dX1 = 0   A32 = dX3/dX2 = 0    A33 = dX3/dX3 = 0   A34 = dX1/dX4 = 1
                A41 = dX4/dX1 = 0   A42 = dX4/dX2 = Kq   A43 = dX4/dX3 = A43 A44 = dX4/dX4 = -gamma

            A43 = -(g/l).cos(X3) = 0 for pendulum up

                | 0     1      0      0  |
            A = | 0    beta    0      0  |
                | 0     0      0      1  |
                | 0     Kq    A43  -gamma|
                
            beta => mechanical loss and damping of the motor-wheels subsystem 
            Kp => 'gain' of input (i.e voltage) to acceleration of the motor-wheels subsystem 
            Kq =>
            Kr => gamma => Damping of the pendulum
        """
        A_matrix = numpy.array([
            [0.0,       1.0,        0.0,        0.0],
            [0.0,       self._beta, 0.0,        0.0],
            [0.0,       0.0,        0.0,        1.0],
            [0.0,       0.0,        self._A43,  -self._gamma]
        ])
        B_matrix = numpy.array([
            [0.], 
            [self._kp], 
            [0.], 
            [self._kr]
        ])
        C_matrix = numpy.array([[1., 1., 1., 1.]])
        D_matrix = numpy.array([[0.]])

        self._m_continuous_system = control.ss(A_matrix, B_matrix, C_matrix, D_matrix)
        
        """
            System checks
                Controllability =>  The system is controllable if the rank of its Controllability matrix B
                                    is equal to the nb of states
                Observability   =>  The system is observable if the rank of its observability matrix C
                                    is equal to the nb o states
        """
        inverted_pendulum_controllable = numpy.linalg.matrix_rank(control.ctrb(A_matrix, B_matrix))
        if A_matrix.shape[1] == inverted_pendulum_controllable:
            message = "The system is controllable, its controllability matrix is full rank " \
                    "(rank = {rank:d}, Nb states = {states:d})"
            print(message.format(rank=inverted_pendulum_controllable, states=A_matrix.shape[1]))
        else:
            message = "The system is NOT controllable, its controllability matrix has rank " \
                    "{rank:d} but numbers of states are {states:d})"
            print(message.format(rank=inverted_pendulum_controllable, states=A_matrix.shape[1]))

        inverted_pendulum_observable = numpy.linalg.matrix_rank(control.obsv(A_matrix, C_matrix))
        if A_matrix.shape[1] == inverted_pendulum_observable:
            message = "The system is observable, its observability matrix is full rank " \
                    "(rank = {rank:d}, Nb states = {states:d})"
            print(message.format(rank=inverted_pendulum_observable, states=A_matrix.shape[1]))
        else:
            message = "The system is NOT observable, its observability matrix has rank " \
                    "{rank:d} but numbers of states are {states:d})"
            print(message.format(rank=inverted_pendulum_observable, states=A_matrix.shape[1]))

            
