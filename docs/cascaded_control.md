\# Cascaded Control



The SDK promotes cascaded control. It is available for use by including:



```cpp

\#include "Controllers/Cascade.h"

```



The control system comes with three implemented stages: `Attitude`, `Velocity` and `Position`, which are propagated (in a full cascade system) as follows:



```

Position > Velocity > Attitude

```



The `Attitude` controller returns the required torque vector in the drone frame for a desired control input — whether that may be a velocity stick input or waypoint-controlled input via position.



The `Orchestrator` resolves the three-stage control and combines the control systems for robust and generalized control. Each control system has a purpose, which is why this division is present.



The orchestrator gets fed pointers to four main modules: the attitude, velocity and position controllers, and the model. It also allows for fragmented or specific control cascades, such as:



```

Velocity > Attitude

Attitude (standalone)

```



This orchestrator still uses some hard-coding, specifically the model via `IRigidModel`. This may be resolved in the future with a more complete virtual model. Each controller supports a set of control flags that can be modified.



---



\## Attitude



All cascaded control systems use some form of looping cascade. In the attitude controller, the outer loop is for orientation, and the inner loop is for rate.



Using PID algorithms, the orientation error is computed. The derivative of angular position is angular velocity, so the outer loop returns the desired angular velocity (rate).



The inner loop then computes the desired angular acceleration (second derivative) using another PID controller.



Since the attitude controller does not get direct access to the model, it cannot compute torque itself — it needs the inertia matrix. The model (via orchestrator) computes the torque using this information.



\### Usage



\*\*Gain Configuration\*\*



```cpp

auto attitudeCtrl = Cascade::Attitude()

&nbsp; .configureOrientationGains(

&nbsp;   PIDGains{.kp = 4.5, .ki = 0.02, .kd = 0.05, .kf = 15},

&nbsp;   PIDGains{.kp = 4.5, .ki = 0.02, .kd = 0.05, .kf = 15},

&nbsp;   PIDGains{.kp = 5.0, .ki = 0.10, .kd = 0.05, .kf = 10}

&nbsp; )

&nbsp; .configureRateGains(

&nbsp;   PIDGains{.kp = 2.5, .ki = 0.05, .kd = 0.02, .kf = 10},

&nbsp;   PIDGains{.kp = 2.5, .ki = 0.05, .kd = 0.02, .kf = 10},

&nbsp;   PIDGains{.kp = 3.5, .ki = 0.02, .kd = 0.01, .kf = 10}

&nbsp; );

```



\*\*Control Flags\*\*



```cpp

Cascade::Attitude::ControlFlags flagsAtt;

flagsAtt.useTiltLimit = true;

flagsAtt.useRateLimit = true;

flagsAtt.holdHeading = false;

flagsAtt.tiltLimitAbs = PI / 4;

flagsAtt.rateLimitAbs = 3.0f;



attitudeCtrl.setControlFlags(\&flagsAtt);

```



\*\*Control Loop\*\*



```cpp

attitudeCtrl.updateMeasuredOrientation(x, y, z);

attitudeCtrl.updateMeasuredRate(x, y, z);

attitudeCtrl.configureOrientationTarget(x, y, z);

attitudeCtrl.configureRateTarget(x, y, z);



Vector3 output = attitudeCtrl.compute(dt);

```



> `Vector3` is a 3D frame coordinate container.  

> All angular units are in \*\*radians\*\*.



You can test this controller using the built-in `mpu6050` module.



---



\## Velocity



The velocity controller does \*\*not\*\* use a cascade structure internally. Instead, it uses PID with a derivative term to compute the required acceleration vector to achieve a target velocity.



This is used for safety and stability. The output acceleration is combined with thrust and trig projections to compute the required orientation, typically in the planar XY frame.



This is orchestrated together using the model.



\### Usage



\*\*Gain Configuration\*\*



```cpp

auto velocityCtrl = Cascade::Velocity()

&nbsp; .configureVelocityGains(

&nbsp;   PIDGains{.kp = 0.8, .ki = 0.00, .kd = 0.20, .kf = 10},

&nbsp;   PIDGains{.kp = 0.8, .ki = 0.00, .kd = 0.20, .kf = 10},

&nbsp;   PIDGains{.kp = 1.0, .ki = 0.05, .kd = 0.20, .kf = 10}

&nbsp; );

```



\*\*Control Flags\*\*



```cpp

Cascade::Velocity::ControlFlags flagsVel;

flagsVel.useVelocityLimit = true;

flagsVel.velocityLimitAbs = 8.0f;

flagsVel.gravity = 9.81f;

flagsVel.yaw\_reference = 0;



velocityCtrl.setControlFlags(\&flagsVel);

```



\*\*Control Loop\*\*



```cpp

velocityCtrl.updateMeasuredVelocity(x, y, z);

velocityCtrl.configureVelocityTarget(x, y, z);



Vector3 output = velocityCtrl.compute(dt);

```



> Velocity is measured in \*\*meters per second (m/s)\*\*.



---



\## Position



🚧 Not yet implemented.



---



\## Cascade Orchestrator



The cascade orchestrator ties together all control systems to eventually compute a final \*\*torque vector\*\*.



It supports combinations such as:



```

Position > Velocity > Attitude

Velocity > Attitude

Attitude (alone)

```



> ⚠️ Leaving out the \*\*Attitude\*\* controller is not allowed. For testing, it must be bridged manually.



The returned torque type is:



```cpp

Cascade::Orchestrator::Torque

```



Which is just an alias for `Vector3`, for clarity.



The orchestrator uses a model that implements `IRigidModel`. The model provides physical parameters like \*\*mass\*\* and \*\*inertia\*\*, and is also used by the mixer and other modules.



\### Usage



\*\*Initialization\*\*



```cpp

auto resolver = Cascade::Orchestrator()

&nbsp; .useModel(\&model4)

&nbsp; .useVelocityInterface(\&velocityCtrl)

&nbsp; .useAttitudeInterface(\&attitudeCtrl);

```



\*\*Compute Output\*\*



```cpp

Cascade::Orchestrator::Torque torque = resolver.compute(dt);

```



---



\## Overview of Structure



Figure 3.1 shows a schematic of the current drone control workflow.



!\[](https://lh7-rt.googleusercontent.com/docsz/AD\_4nXfiZ-jtWeIxQfEBBCBdS-pDE8QcB2\_hJyjVabx2VmiOlvkFbOFQqClQgIpRI4MfIdStFiuY3QwUr3FbIeEiR2frWewzaxePSKrT7rdoBzJEmDY0f\_ko82lUNzGqI\_MTUAwPkKBK?key=v6nN2IgIJliGnaRhDJR8WG25)



\_Figure 3.1. Cascaded Control schematic.\_





