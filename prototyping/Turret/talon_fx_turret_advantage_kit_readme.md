# TalonFX Turret – WPILib + AdvantageKit

This document describes the **design, architecture, and implementation** of a **robot turret subsystem** driven by a **single TalonFX (Falcon / Kraken)** motor.  
The turret is responsible for **rotational aiming** of the shooter and supports **manual control**, **command-based closed-loop control**, and **full simulation**, all implemented using **WPILib**, **CTRE Phoenix 6**, and **AdvantageKit**.

---

## 1. Problem Definition & Goals

The turret provides precise angular positioning to align the shooter with field targets. Unlike drivetrain mechanisms, the turret:

- Operates in a **bounded angular range**
- Must be **repeatable and accurate**
- Often integrates with vision and superstructure logic

### Design Requirements

- Absolute or relative angle control (degrees)
- Software-enforced soft limits
- Manual (open-loop) control for testing and fallback
- Command-based (closed-loop) position control
- AdvantageKit logging and replay support
- Identical behavior in real and simulated robots

---

## 2. Architecture Overview

The turret follows the standard **AdvantageKit IO abstraction pattern**, mirroring other subsystems such as the shooter.

```
Turret (Subsystem)
 ├── TurretIO (interface)
 │    ├── TurretIOInputs (AutoLogged)
 │
 ├── TurretReal (TalonFX hardware)
 ├── TurretSim  (SingleJointedArmSim-based)
 │
 └── TurretState (desired turret behavior)
```

### Architectural Intent

| Component | Responsibility |
|---------|---------------|
| `Turret` | Control logic, limits, logging |
| `TurretIO` | Hardware abstraction contract |
| `TurretReal` | CTRE TalonFX position control |
| `TurretSim` | Physics-based angular simulation |
| `TurretState` | High-level turret intent |

---

## 3. TurretState – Defining Intent

The turret operates on **high-level intent states**, rather than raw motor commands.

```java
public enum TurretState {
  IDLE(0.0),
  ZERO(0.0),
  FORTYFIVE(45.0),
  NINETY(90.0),
  ONEEIGHTY(180.0);
  TWOTWENTYFIVE(225);
  TWOSEVENTY(270.0);
  THREESIXTY(360.0);

  public final double angleDeg;

  TurretState(double angleDeg) {
    this.angleDeg = angleDeg;
  }
}
```

### Why States?

- Centralized angular targets
- Easier tuning and debugging
- Clean command interfaces
- Natural extension point for vision-based offsets

---

## 4. TurretIO – Hardware Contract

The IO interface defines the **exact data and controls** the subsystem requires.

```java
public interface TurretIO {

  @AutoLog
  class TurretIOInputs {
    public double positionDeg = 0.0;
    public double velocityDegPerSec = 0.0;

    public double appliedVolts = 0.0;
    public boolean motorConnected = false;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  default void setVoltage(double volts) {}
  default void setPosition(double angleDeg) {}

  default void stop() {}
}
```

---

## 5. TurretReal – TalonFX Implementation

The real turret implementation uses **CTRE Phoenix 6 position control**.

```java
public class TurretReal implements TurretIO {

  private final TalonFX motor = new TalonFX(20);
  private final PositionVoltage positionRequest = new PositionVoltage(0);

  public TurretReal() {
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
  }

  @Override
  public void setPosition(double angleDeg) {
  }

  @Override
  public void stop() {
  }
}
```

---

## 6. TurretSim – Angular Simulation

Simulation uses WPILib’s `SingleJointedArmSim`, which maps well to turret dynamics.

```java
public class TurretSim implements TurretIO {

  private final SingleJointedArmSim sim = new SingleJointedArmSim(
      DCMotor.getFalcon500(1),
      100.0,
      0.05,
      0.25,
      Units.degreesToRadians(-180),
      Units.degreesToRadians(180),
      true);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    sim.update(0.02)l  }

  @Override
  public void setVoltage(double volts) {
  }

  @Override
  public void stop() {
  }
}
```

---

## 7. Turret Subsystem – Core Logic

The subsystem arbitrates between **manual control** and **state-based control**, while enforcing limits and logging.

```java
public class Turret extends SubsystemBase {

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs =
      new TurretIOInputsAutoLogged();

  private TurretState desiredState = TurretState.IDLE;
  private boolean manualMode = false;
  private double manualVolts = 0.0;

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
  }

  public void setManualVoltage(double volts) {
  }

  public void exitManualMode() {
  }

  public void setState(TurretState state) {
  }

  public boolean atSetpoint() {
    return Math.abs(inputs.positionDeg - desiredState.angleDeg) < 2.0;
  }
}
```

---

## 8. Manual Control

Manual mode is used for:

- Zeroing and calibration
- Pit testing
- Emergency recovery

Example binding:

```java
new RunCommand(
    () -> turret.setManualVoltage(-driver.getRightX() * 6.0),
    turret)
```

---

## 9. Command-Based Turret Control

Commands request **intent**, not voltage.

```java

  /**
   * Command that sets the turret to a desired state and finishes
   * once the turret reaches its setpoint.
   */
  public Command setStateCommand(TurretState state) {
    return this.runOnce(() -> setState(state))
        .andThen(
            Commands.waitUntil(this::atSetpoint))
        .withName("Turret:SetState:" + state.name());
  }

  /**
   * Command that continuously holds a turret state
   * (useful for default or parallel commands).
   */
  public Command holdStateCommand(TurretState state) {
    return this.run(() -> setState(state))
        .withName("Turret:HoldState:" + state.name());
  }
```

---

## 10. Outcomes & Lessons Learned

### What Worked Well

- Position-based control simplified aiming logic
- AdvantageKit replay enabled tuning without hardware
- State-based design made vision integration trivial

### Best Practices Reinforced

- Enforce limits in the subsystem, not commands
- Log desired vs actual angles
- Keep manual control isolated

---

## Final Takeaway

This turret architecture cleanly supports:

- Manual operator control
- Deterministic autonomous aiming
- Vision-based offsets
- Simulation-first development

while remaining **robust, testable, and competition-safe**.

---

**Recommended Extensions**

- Vision angle offsets (Limelight / PhotonVision)
- Continuous angle wrapping
- Motion Magic / ProfiledPID
- Superstructure coordination with shooter

