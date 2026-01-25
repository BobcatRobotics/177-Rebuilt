# Shooter VelocityTorqueFOC Tuning (CTRE Phoenix 6 + AdvantageScope)

This guide explains how to tune an **FRC shooter** using **VelocityTorqueFOC** on Falcon / Kraken motors, with **AdvantageKit logging** and **AdvantageScope visualization**.

The focus is **shot consistency and recovery**, not just spin-up time.

---

## What VelocityTorqueFOC Does

- **Setpoint**: Velocity (RPS or RPM)
- **Control Loop**: Velocity PID
- **Output**: Torque (FOC current)
- **Why it’s good**: Fast recovery, consistent shots under load

---

## Required Logged Signals

Log these at **50–100 Hz**:

| Signal | Description |
|-----|-------------|
| `Shooter/VelocityRPS` | Measured flywheel speed |
| `Shooter/VelocitySetpointRPS` | Target speed |
| `Shooter/StatorCurrent` | Motor current (torque proxy) |
| `Shooter/AppliedTorque` | Optional (if available) |

Example (AdvantageKit):
```java
log.recordOutput("Shooter/VelocityRPS", velocity);
log.recordOutput("Shooter/VelocitySetpointRPS", setpoint);
log.recordOutput("Shooter/StatorCurrent", statorCurrent);
log.recordOutput("Shooter/AppliedTorque", appliedTorque);
```

---

## Plot 1: Velocity vs Setpoint (Spin-Up Response)

**What this plot shows**
- How fast the shooter reaches speed
- Overshoot and oscillation
- Overall loop stability

**Good Response**
```
Setpoint:  ──────────────────
Velocity:   /────────────────
```

**Problems & Fixes**

| Symptom | Cause | Fix |
|------|------|----|
| Slow spin-up | Low P or bad FF | Increase `kP` or `kV` |
| Overshoot | P too high | Lower `kP`, add `kD` |
| Oscillation | P too high | Reduce `kP`, add `kD` |

---

## Plot 2: Velocity Error

**What this plot shows**
- How aggressively error is corrected
- Stability of the control loop

**Good Response**
```
Error:  \______
```

**Bad Patterns**

| Pattern | Meaning | Fix |
|------|------|----|
| Error never reaches zero | Missing FF or I | Fix `kV`, add small `kI` |
| Oscillating error | P too high | Lower `kP` |
| Error spikes on shots | Normal — watch recovery | Tune P/D |

---

## Plot 3: Torque / Current Output

**Why this matters**
This plot shows what the motor controller is *doing* to maintain velocity.

**Good Pattern**
```
Torque:  /\____/\____
```

- Large startup spike
- Low steady-state torque
- Short spikes when shooting

**Bad Patterns**

| Pattern | Meaning | Fix |
|------|------|----|
| Constant high current | FF too low | Increase `kV` |
| Oscillating current | P too high | Reduce `kP` |
| Slow torque ramp | P too low | Increase `kP` |

---

## Plot 4: Shot Disturbance Recovery (MOST IMPORTANT)

**How to capture**
1. Spin shooter to target velocity
2. Fire notes
3. Zoom in on each shot

**Good Response**
```
Velocity:  ───\__/───
Torque:        /\    
```

**Target Metrics**
- RPM dip is small
- Recovery < **200 ms**
- No overshoot after recovery

**Tuning From This Plot**

| Issue | Fix |
|----|----|
| Big RPM drop | Increase `kP` or FF |
| Slow recovery | Increase `kP` |
| Overshoot after shot | Add `kD` |
| Speed creeps upward | Reduce `kI` |

---

## Optional Plot: Feedforward Validation

**Goal**
At constant velocity, applied torque should be **flat**.

If torque rises with speed → `kV` is incorrect.

---

## Recommended AdvantageScope Layout

**Stacked View**
1. Velocity vs Setpoint
2. Velocity Error
3. Torque / Current

Enable:
- Separate layouts for:
  - Spin-up
  - Shot recovery

---

## Tuning Priority Order

1. Feedforward (`kS`, `kV`)
2. Proportional (`kP`)
3. Derivative (`kD`)
4. Integral (`kI`, only if needed)

> **Golden Rule:** Tune for shot recovery, not spin-up time.

---

## Notes for Multi-Wheel Shooters

- Tune **main flywheel first**
- Backspin wheels typically need less torque and lower gains

---
