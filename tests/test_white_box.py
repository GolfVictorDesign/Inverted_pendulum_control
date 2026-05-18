import time
import pytest

from Inverted_pendulum_lqg import inverted_pendulum_robot

class TestRobotWhiteBox:
    
    @pytest.fixture(scope="class", params=[
        # (0.0,       0.0,        0.0,        0.0),
        # (-0.1,      0.0,        0.0,        0.0),
        # (0.0,       0.00001,    0.0,        0.0),
        # (0.0,      -0.00001,    0.0,        0.0),
        # (0.0,       0.0,        0.00001,    0.0),
        # (0.0,       0.0,       -0.00001,    0.0),
        # (0.0,       0.0,        0.0,        0.00001),
        # (0.0,       0.0,        0.0,       -0.00001),
        # (0.00001,  -0.00001,    0.0,        0.0),
        # (-0.00001,  0.00001,    0.0,        0.0),
        # (0.0,       0.0,        0.00001,    0.00001),
        # (0.0,       0.0,       -0.00001,   -0.00001),
        # (0.0,       0.0,        0.00001,   -0.00001),
        (0.0,       -0.001,        0.00001,   -0.00001),
    ])
    def fixture_inverted_pendulum(self, request):
        
        x           = request.param[0]
        x_dot       = request.param[1]
        theta       = request.param[2]
        theta_dot   = request.param[3]
        
        yield inverted_pendulum_robot(x, x_dot, theta, theta_dot)
        
    def test_plot(self, fixture_inverted_pendulum:inverted_pendulum_robot):
        fixture_inverted_pendulum.plot_dynamics()
        
        
    def test_animation(self, fixture_inverted_pendulum:inverted_pendulum_robot):
        fixture_inverted_pendulum.animate_system()
        
        
                