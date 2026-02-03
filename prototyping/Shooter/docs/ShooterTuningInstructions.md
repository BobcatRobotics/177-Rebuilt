# Step-by-Step Shooter Tuning for (VelocityTorqueFOC)

This guide is written for high school FRC teams to **safely tune a shooter** using VelocityTorqueFOC.

---

## Prep Before Tuning

1. **Safety First**
   - Ensure the robot is secured (test rig or wheels off the floor).
   - Remove loose objects near the flywheel.

2. **Check Hardware**
   - Motor wires secure.
   - Gear ratio matches code.
   - Firmware up to date (Phoenix 6).

3. **Set Up AdvantageKit/AdvantageScope**
   - Enable logging.
   - Log these signals:

      | Signal | Description |
      |-----|-------------|
      | `Shooter/VelocityRPS` | Measured flywheel speed |
      | `Shooter/VelocitySetpointRPS` | Target speed |
      | `Shooter/StatorCurrent` | Motor current (torque proxy) |
      | `Shooter/AppliedTorque` | Optional (if available) |

---

## Step 1: Start With Feedforward (FF)

1. Set PID gains to 0:
```java
kP = 0
kI = 0
kD = 0
```
2. Slowly ramp the shooter from 0 → target speed (e.g., 1200 RPM).
3. Observe torque/current:
   - `kS` = torque needed to start spinning.
   - `kA` = inertia feed forward, used to calculate the amount of motor torque or voltage required to achieve the desired accelration. Improves the responsiveness of the velocity by proactively commanding torque based on acceleration rather than waiting for error to build up.
   - `kV` = torque needed to hold speed.
4. Write down FF numbers.

---

## Step 2: Tune Proportional (P)

1. Set `kP` small, `I` and `D` = 0.
2. Command 0 → target RPM.
3. Observe velocity vs setpoint plot.
4. Increase `kP` until:
   - Spin-up is fast.
   - Slight overshoot may occur.
5. Stop before oscillation.
> Back off ~20% from the value causing oscillation.

---

## Step 3: Add Derivative (D)

1. Set small `kD` (0.001–0.01).
2. Observe overshoot:
   - Decrease → good.
   - Response too slow → reduce `kD`.

---

## Step 4: Add Integral (I) Only If Needed

- Usually not needed if FF is correct.
- Only add tiny I if shooter never reaches target velocity.
- Too much I → velocity climbs after a shot.

---

## Step 5: Test Under Load (Fire Balls)

1. Spin shooter to target velocity.
2. Fire a ball.
3. Observe plots:
   - Velocity dips briefly → normal.
   - Recovery < 200 ms.
4. Adjust:
   - Big dip → increase `kP` or FF.
   - Slow recovery → increase `kP`.
   - Overshoot → add `kD`.

---

## Step 6: Safety and Limits

- Set current limits:
```java
config.CurrentLimits.StatorCurrentLimit = 40;
config.CurrentLimits.StatorCurrentLimitEnable = true;
```
- Ensure no contact with hands, tools, or wires during testing.

---

## Step 7: Validate and Repeat

- Test multiple times.
- Check spin-up, steady velocity, and recovery.
- Record final gains in the code, save in github.

---

## Tips for Students

- Work in pairs: one controls robot, one monitors plots.
- Log all runs.
- Only change one gain at a time.

---





🎯 How to Tune kV in 60 Seconds

Disable PID (P = I = D = 0)

Apply only FF

Slowly increase setpoint

Watch:

If actual speed < setpoint → increase kV

If actual speed > setpoint → decrease kV

kS is right when:

Wheel just starts spinning cleanly

No buzzing / hesitation at low speed