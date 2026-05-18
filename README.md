# Inverted_pendulum_lqg

Build with the help of Steve Brunton's Control Bootcamp :
<https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m>
and David Deley's inverted pendulum studies :
<https://daviddeley.com/pendulum/pendulum.htm>
and <https://www3.diism.unisi.it/~control/ctm/examples/pend/invpen.html>

## State-space model

The system is a self balancing robot as we can see many on the internet.

It has 2 brushed DC motors with an IMU on top to measure \thetaand encoders on the motors
The DC motors are controlled by a H-bridge and PWM.

The states representing the system:

* x                => Position of the motor-wheels subsystem
* x'               => Linear speed of the motor-wheels subsystem
* \theta           => Angle of the pendulum
* \theta           => Angular velocity of the pendulum

| States space derivatives | States space variables |  States |                  Description               | Fixed points  |
|:------------------------:|:----------------------:|:-------:|:------------------------------------------:|:-------------:|
|            X1'           |           X1           |   x     |   Position of the motor-wheels subsystem   |   arbitrary   |
|            X2'           |           X2           |   x'    | Linear speed of the motor-wheels subsystem |       0       |
|            X3'           |           X3           |   \theta|              Angle of the pendulum         |     0 (up)    |
|            X4'           |           X4           |   \theta|       Angular velocity of the pendulum     |       0       |

Equations of movements:

System's parameters:

* m = mass of the body
* M = mass of the cart
* Mw = mass of the wheels
* Mt = total mass = $M + Mw + m$
* l = length from the extremity to the centre of mass
* Km = motor's constant
* Ri = motor's inductor/H bridge resistance
* Ip = pendulum's inertia = $\frac {m \cdot l²}{3}$ (homogenous rod) or $m \cdot l²$ (mass on a weightless rod)
* Ic = cart's inertia = $M \cdot  \ddot x$
* Im = motor's inertia
* Rwe = external radius of the wheels
* Rwi = internal radius of the wheels
* Iw = wheels' inertia = $0.5 \cdot Mw \cdot (Rwe²+Rwi²)$
* J = Ic + Im + Iw
* Tm = motor torque = $( \frac {Km} {Rwe \cdot Ra}) \cdot u - ( \frac {Km²} {Rwe²-Ra}) \cdot x'$
* Bm = motor's damping coefficient or mechanical losses
* Bc = cart/wheels damping coefficient or mechanical losses
* B = combined damping coefficients (motor + cart) = $ \frac {Km²}{JRa} + \frac {Bm}J + \frac {Bc}J \cdot r²$
* Brot = viscous rotational friction coefficient ≈ 0 as first estimation
* Ff = viscous friction forces = $B \dot x$
* phi = motor angular position
* Kp = pendulum horizontal displacement coefficient = $Rwe \cdot \phi$
* D = determinant function of angle = $J \cdot (Ip + ml²) - (m \cdot l \cdot rwe \cdot cos\ \theta)²$

$\ddot x = ((Ip + ml²) \cdot (Tm - Ff + m \cdot l \cdot \dot {\theta}² \cdot sin\ \theta) - (m \cdot l \cdot Rwe \cdot cos\ \theta) \cdot (m \cdot g \cdot l \cdot sin\ \theta- Brot \cdot \dot \theta)) / D$

$\ddot \theta  = ((-m \cdot l \cdot Rwe \cdot cos \theta) \cdot (Tm - Ff + m \cdot l \cdot \dot {\theta²} \cdot sin \theta) + J \cdot (m \cdot g \cdot l \cdot sin \theta - Brot \cdot \dot \theta)) / D$
