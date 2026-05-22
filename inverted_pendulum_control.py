"""
Created on 2022-06-18
@author: GolfVictorDesign

Simulation of a LQG control for inverted pendulum on wheels
"""
import numpy
import control
import matplotlib.pyplot as plot
import matplotlib.animation as animation


class InvertedPendulumRobot:
    
    # Physical constant parameters
    _m_sys_params = {
        "m"     : 0.20,     # m, mass of the pendulum
        "M"     : 0.30,     # M, mass of the cart
        "Mw"    : 0.05,     # Mw  mass of the wheel
        "l"     : 0.15,     # l, arm length from the cart to the center of mass of the pendulum
        "g"     : 9.81,     # gravity constant    
        "Km"    : 2400 / (60 * 2 * numpy.pi * 12),   # Motor constant Km (r/V.sec) 2400rpm@12v
        "Ri"    : 0.1,      # motor inductor resistance
        "N"     : 12,       # Motor gear ratio
        "eta"   : 0.7,      # gear efficiency
        "Rwe"   : 0.05,     # Wheel external radius Rwe (m)
        "Rwi"   : 0.04,     # Wheel internal radius Rwi (m)
        "J"     : 0.1,
        "Brot"  : 0.0,      # Motor frictions
        "Bc"    : 0.01      # Cart/wheels frictions and damping factor
    }
    
    # System plot representations
    _m_y_labels_ode     = (("Position x","m"), ("Speed x'","m/s"), ("Position Theta","rad"), ("Speed Theta'","rad/s"))

    def __init__(
            self,
            motor_Bm = 0.0,
            integration_time = 10.0,
            dt_system = 0.001,
            x_init = 0.0, 
            x_dot_init = 0.0, 
            theta_init = 0.0, 
            theta_dot_init = 0.0,
            show_anim=False
    ) -> None:
        """ 
            Initialize and integrate the system model
        """
        # Setup of initial values
        self._m_sys_params["Bm"] = motor_Bm
        self._m_initial_states = numpy.array([x_init, x_dot_init, theta_init, theta_dot_init])
        self._m_system_sampling_rate = dt_system 
        self._m_integration_time = integration_time
        
        # Mt => total mass
        self._m_sys_params["Mt"] =  self._m_sys_params["m"] +  self._m_sys_params["M"] +  \
                                               self._m_sys_params["Mw"] 
                                               
        # Inertia of an arm around it extremity
        self._m_sys_params["Ip"] = (1/3) * self._m_sys_params["m"] * (self._m_sys_params["l"]**2)
        
        # Motor inertia
        self._m_sys_params["Im"] = 0.1 * self._m_sys_params["N"]**2      
        
        # Inertia of the wheel
        self._m_sys_params["Iw"] =  0.5 * self._m_sys_params["Mw"] * (self._m_sys_params["Rwe"]**2 + \
                                        self._m_sys_params["Rwi"]**2)
        
        # System definitions and solution computation
        self._m_non_linear_system = control.nlsys(
                                                self.__inverted_pendulum_update_model, 
                                                self.__inverted_pendulum_output_model,
                                                states = ["x", "x_dot", "theta", "theta_dot"],
                                                inputs = ["Ui"],
                                                outputs = ["x", "x_dot", "theta", "theta_dot"],
                                                params = self._m_sys_params)
        
        self._m_lti_system = control.linearize(
                                            self._m_non_linear_system,
                                            [0.0, 0.0, 0.0, 0.0],
                                            [0.0])
        
        self._m_lti_discrete_system = control.sample_system( self._m_lti_system, self._m_system_sampling_rate )
        
        self._m_tf = self._m_lti_system.to_tf()
        self._m_discrete_tf = self._m_lti_discrete_system.to_tf()
        
        print(self._m_lti_system)
        print()
        print(self._m_lti_discrete_system)
        print()
        print(self._m_tf)
        print()
        print(self._m_discrete_tf)
        
        # Animation plotting setup
        self._m_figure = None
        if show_anim == True:
            self._m_circle_patch = plot.Circle((self._m_initial_states[0], 0), self._m_sys_params["Rwe"] * 10, fc="b")
            #                       [[x1,  y1 ],    [x2,   y2 ],    [x3,  y3  ],   [x4,   y4  ]]
            body    = numpy.array(  [[-0.025, 0.0], [-0.025, 0.20], [0.025, 0.20], [0.025, 0.0]]) 
            self._m_body_patch = plot.Polygon(body*10, fc="y")
                
            self._m_figure = plot.figure(figsize=(10,10))
            self._m_axes = self._m_figure.add_subplot(autoscale_on=False, xlim=(-5, 5), ylim=(-1, 9))
            self._m_axes.grid(True, 'major', linestyle = "-")
            self._m_axes.set(xlabel="X position")
            self._m_axes.plot([0, 0], [1,1], lw=2)
         
    def __inverted_pendulum_update_model(self, t, X, u, params):
        
        # states
        x           = X[0]
        x_dot       = X[1]
        theta       = X[2]
        theta_dot   = X[3]
        
        # Get the parameters values
        m, M, Mw, l, g, Km, Ri, Im, Rwe, J, Ip, Bm, N, eta, Brot, Bc = map(
            params.get, 
            [
                "m", "M", "Mw", "l", "g", "Km", "Ri", "Im", "Rwe", "J", "Ip", "Bm", "N", "eta", "Brot", "Bc"
            ])
        
        # inputs
        voltage = u[0]
        
        """
            Motor Torque
            Tm  = (Km / (Rwe.Ra)).u - (Km² / (Rwe²-Ra)).x'
                | Torque generated  | counter torque generated |
                |    by current     |       By back EMF        |  
            The second term is avoided when u = 0 => open circuit => no current flow => no breaking
        """
        Tm      = (Km / (Rwe * Ri)) * voltage - (Km**2 / (Rwe**2 - Ri)) * x_dot if voltage > 0 else \
                    (Km / (Rwe * Ri)) * voltage
                    
        """
            Torque on wheel
            Tw = Tm * N * eta
        """
        Tw = Tm * N * eta
        
        """
            Breaking/damping factor
            B = (KmN)²/JRa     +     Bm/J      +   (Bc/J).Rwe²
                |  Back EMF     |     motor     |    rotational     |
                |  damping      |    damping    |   cart damping    |
            
            Breaking/damping force
            Ff = B.x'
        """      
        B  =   ((Km * N)**2 / (J * Ri)) + (Bm / J) + ((Bc / J) * Rwe**2)
        Ff  =   B * x_dot 
        
        """
            Determinant
            D =     J.(Ip + ml²)    -   (m.l.rwe.cos(theta))²
                |   rotational      |   cinematic coupling      | 
                |   System inertia  |       cart/body           |
        """
        D =     J * (Ip + (m * l**2)) - (m * l * Rwe * numpy.cos(theta))**2
        
        """
            Linear acceleration of the cart on x axis
            x" = ((Ip + ml²). (Tw - Ff + m.l. theta'².sin(theta)) - (m.l.Rwe. \
                    cos(theta)).(m.g.l.sin(theta) - Brot.theta')) / D
        """
        x_ddot = ((Ip + m * l**2) * (Tw - Ff + m * l * theta_dot**2 * numpy.sin(theta)) - (m * l * Rwe * \
                    numpy.cos(theta)) * ((m * g * l * numpy.sin(theta)) - (Brot * theta_dot))) / D
        
        """
            Angular acceleration on pendulum
            Theta" = (-(m.l.Rwe.cos(theta)). (Tw - Ff + m.l. theta'².sin(theta)) + \
                        J.(m.g.l.sin(theta) - Brot.theta')) / D
        """
        theta_ddot  = (-(m * l * Rwe * numpy.cos(theta)) * (Tw - Ff + (m * l * theta_dot**2 * numpy.sin(theta))) + \
            J * ((m * g * l * numpy.sin(theta)) - (Brot * theta_dot))) / D                                    
                                        
        # Cart linear inertia
        Ic = abs(M * x_ddot)
        
        # J = Ic + Im + Iw + MRwe²
        J = Ic + Im + (Mw * x_ddot) + (M * Rwe**2)

        return numpy.array([x_dot, x_ddot, theta_dot, theta_ddot])
    
    def __inverted_pendulum_output_model(self, t, X, u, params):
        # full state
        return numpy.array([X[0], X[1], X[2], X[3]])

    def step_response(self):
        time_steps = numpy.arange(0, self._m_integration_time, 0.1)
        self._m_nl_step_response = control.step_response(self._m_non_linear_system, time_steps)
        self._m_nl_step_response.plot(plot_inputs='overlay')
        
        return self   
    
    def initial_state_response(self):
        time_steps = numpy.arange(0, self._m_integration_time, 0.1)
        self._m_init_response = control.initial_response(self._m_non_linear_system, time_steps, self._m_initial_states)
        self._m_init_response.plot()
        
    def get_initial_states(self):
        return self._m_initial_states        
            
    def __init_animation(self):
        self._m_axes.add_patch(self._m_circle_patch)
        self._m_axes.add_patch(self._m_body_patch)
        
        return self._m_body_patch, self._m_circle_patch
    
    def __animation_function(self, frame):
        body_vertices = self._m_body_patch.get_xy()
        for index, _ in enumerate(body_vertices):
            # x
            """body_vertices[index][0] = response.states[0][frame] + \
                                        body_vertices[index][0] * numpy.sin(response.states[2][frame])
            
            # y 
            body_vertices[index][1] *= numpy.cos(response.states[2][frame])"""
            body_vertices[index][0] += 0.1 
            """response.states[0][frame * int((len(response.t) * \
                                                                                sampling))]"""
        
        #patch.set_xy(body_vertices)
        
        wheel_center_x, wheel_center_y = self._m_circle_patch.center
        wheel_center_x += 0.1 #response.states[0][frame * int((len(response.t) * sampling))]
        self._m_circle_patch.center = wheel_center_x, wheel_center_y
        
        return self._m_body_patch, self._m_circle_patch
        
    def animate_system(self) -> None:
        if self._m_figure is not None:
            anim = animation.FuncAnimation(
                                    self._m_figure, 
                                    func=self.__animation_function,
                                    init_func=self.__init_animation,
                                    frames= int(self._m_integration_time / (1000/1000)), 
                                    interval=100, 
                                    blit=True )
            
            plot.title("Inverted pendulum robot animation")
            anim.save("./toto.mp4")


