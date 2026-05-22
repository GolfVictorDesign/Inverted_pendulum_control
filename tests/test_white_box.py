import time
import pytest

from inverted_pendulum_control import InvertedPendulumRobot

class TestRobotWhiteBox:
    
    @pytest.fixture(scope="class", params=[
        (0.01, 0.001, 10.0),
    ])
    def fixture_inverted_pendulum(self, request):
        
        Bm          = request.param[0]
        dt          = request.param[1]
        int_time    = request.param[2]
        
        yield InvertedPendulumRobot(motor_Bm=Bm, dt_system=dt, integration_time=int_time)
        
    def test_plot(self, fixture_inverted_pendulum:InvertedPendulumRobot):
        fixture_inverted_pendulum.step_response()
        
        
    def test_animation(self, fixture_inverted_pendulum:InvertedPendulumRobot):
        fixture_inverted_pendulum.animate_system()
        
        
                