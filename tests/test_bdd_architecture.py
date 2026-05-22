
from pytest_bdd import scenario, given, when, then
from inverted_pendulum_control import InvertedPendulumRobot
    
@scenario('./features/system_requirements.feature', 'The software shall follow this scenario')
def test_something():
    pass

@given("Something", target_fixture="something")
def given():
    return InvertedPendulumRobot()
    
@when("Something happen")
def instantiate_class(something:InvertedPendulumRobot):
    pass
    
@then("I want this outcome")
def create_instance(something:InvertedPendulumRobot):
    print(something.get_initial_states())