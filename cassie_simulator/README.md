# Cassie Featherstone Model
Cassie Simulator using Recursive Newton Euler algorithms from Roy Featherstone.

run "cassie\startup.m" and then "cassie\run_cassie.m"

## Cassie coordinate system

#### Base DOFs
* `q(1)` - x
* `q(2)` - y
* `q(3)` - z
* `q(4)` - yaw
* `q(5)` - pitch
* `q(6)` - roll

#### Joints
* `q(7)` - Left hip abduction
* `q(8)` - Right hip abduction
* `q(9)` - Left hip rotation
* `q(10)` - Right hip rotation
* `q(11)` - Left hip flexion
* `q(12)` - Right hip flexion
* `q(13)` - Left knee joint
* `q(14)` - Right knee joint
* `q(15)` - Left knee spring (constrained to 0)
* `q(16)` - Right knee spring (constrained to 0)
* `q(17)` - Left ankle joint (constrained to q12 = deg2rad(13) - q10)
* `q(18)` - Right ankle joint (constrained to q19 = deg2rad(13) - q17)
* `q(19)` - Left toe joint
* `q(20)` - Right toe joint
