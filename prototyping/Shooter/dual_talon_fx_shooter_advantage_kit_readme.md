# Dual TalonFX Shooter – WPILib + AdvantageKit

This document describes the **design, architecture, and implementation** of an FRC shooter subsystem using **two independently controlled TalonFX motors (Falcon / Kraken)**.  
The system is built using **WPILib**, **CTRE Phoenix 6**, and **AdvantageKit**, and supports **both manual control and command-based control** with full simulation support.

---

## 1. Problem Definition & Goals

The shooter is a critical scoring mechanism that requires **independent control** of two flywheel motors:

- **Top motor** – primary exit velocity and backspin
- **Bottom motor** – trajectory shaping and consistency

### Design Requirements

- Independent RPM control for each motor
- Manual (open-loop) control for testing and fallback
- Command-based (closed-loop) control for match play
- AdvantageKit-first architecture (logging, replay, sim parity)
- Identical subsystem logic across real and simulated robots

---

## 2. Architecture Overview

The shooter follows the standard **AdvantageKit IO abstraction pattern**.

```
Shooter (Subsystem)
 ├── ShooterIO (interface)
 │    ├── ShooterIOInputs (AutoLogged)
 │
 ├── ShooterReal (TalonFX hardware)
 ├── ShooterSim  (FlywheelSim-based)
 │
 └── ShooterState (desired shooter behavior)
```

### Architectural Intent

| Component | Responsibility |
|---------|---------------|
| `Shooter` | Control logic, state management, logging |
| `ShooterIO` | Hardware abstraction contract |
| `ShooterReal` | CTRE TalonFX implementation |
| `ShooterSim` | Physics-based flywheel simulation |
| `ShooterState` | High-level shooter behavior definition |

---

## 3. ShooterState – Defining Behavior

Rather than commands directly controlling motors, the shooter operates on **high-level states**.

```java
public enum ShooterState {
  IDLE(0.0, 0.0),
  HUB(1800.0, 1200.0),
  MID(5200.0, 4800.0),
  FAR(6000.0, 5600.0);

  public final double topRPM;
  public final double bottomRPM;

  ShooterState(double topRPM, double bottomRPM) {
    this.topRPM = topRPM;
    this.bottomRPM = bottomRPM;
  }
}
```

### Benefits

- Centralized tuning
- Easy logging & replay
- Clean command-based API
- Prevents motor control logic from spreading into commands

---

## 4. ShooterIO – Hardware Contract

The IO interface defines **exactly what the subsystem needs** from the hardware layer.

```java
public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double topVelocityRPM = 0.0;
    public double bottomVelocityRPM = 0.0;

    public double topAppliedVolts = 0.0;
    public double bottomAppliedVolts = 0.0;

    public boolean topConnected = false;
    public boolean bottomConnected = false;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setTopVoltage(double volts) {}
  default void setBottomVoltage(double volts) {}

  default void setTopVelocity(double rpm) {}
  default void setBottomVelocity(double rpm) {}

  default void stop() {}
}
```

### Key Properties

- Identical API for sim and real
- Automatic AdvantageKit logging
- No hardware dependencies in the subsystem

---

## 5. ShooterReal – TalonFX Implementation

The real implementation uses **CTRE Phoenix 6 velocity control**.

```java
public class ShooterReal implements ShooterIO {

  private final TalonFX topMotor = new TalonFX(10);
  private final TalonFX bottomMotor = new TalonFX(11);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public ShooterReal() {
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
  }

  @Override
  public void setTopVelocity(double rpm) {
  }

  @Override
  public void setBottomVelocity(double rpm) {
  }

  @Override
  public void stop() {
  }
}
```

---

## 6. ShooterSim – Flywheel Simulation

Simulation mirrors real behavior closely enough to allow command testing and log replay.

```java
public class ShooterSim implements ShooterIO {

  private final FlywheelSim topSim =
      new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.025);

  private final FlywheelSim bottomSim =
      new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.025);

  private double topVolts = 0.0;
  private double bottomVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
  }

  @Override
  public void setTopVoltage(double volts) {
  }

  @Override
  public void setBottomVoltage(double volts) {
  }

  @Override
  public void stop() {
  }
}
```

---

## 7. Shooter Subsystem – Core Logic

The `Shooter` subsystem:

- Owns the current `ShooterState`
- Arbitrates manual vs command-based control
- Logs all inputs and outputs
- Remains hardware-agnostic

```java
public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs =
      new ShooterIOInputsAutoLogged();

  private ShooterState desiredState = ShooterState.IDLE;
  private boolean manualMode = false;

  private double manualTopVolts = 0.0;
  private double manualBottomVolts = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    }
  }

  public void setManualVoltage(double top, double bottom) {
  }

  public void exitManualMode() {
  }

  public void setState(ShooterState state) {
  }

  public boolean atSetpoint() {
    return Math.abs(inputs.topVelocityRPM - desiredState.topRPM) < 100
        && Math.abs(inputs.bottomVelocityRPM - desiredState.bottomRPM) < 100;
  }
}
```

---

## 8. Manual Control

Manual control is intended for:

- Pit testing
- Motor verification
- Emergency fallback

Example binding:

```java
new RunCommand(
    () -> shooter.setManualVoltage(
        -driver.getRightY() * 12.0,
        -driver.getLeftY() * 12.0),
    shooter)
```

Manual mode bypasses closed-loop control but **retains full logging**.

---

## 9. Command-Based Control

Commands request **states**, not motor outputs.

```java
  /**
   * Sets the shooter to a desired state and completes
   * once both flywheels reach their target RPMs.
   */
  public Command setStateCommand(ShooterState state) {
    return this.runOnce(() -> setState(state))
        .andThen(
            Commands.waitUntil(this::atSetpoint))
        .withName("Shooter:SetState:" + state.name());
  }

  /**
   * Continuously holds a shooter state.
   * Useful for parallel or default commands.
   */
  public Command holdStateCommand(ShooterState state) {
    return this.run(() -> setState(state))
        .withName("Shooter:HoldState:" + state.name());
  }
```

Usage example:

```java
new SetShooterState(shooter, ShooterState.SPEAKER)
    .andThen(feedCommand);
```

---

## 10. Outcomes & Lessons Learned

### What Worked Well

- Independent motor tuning improved shot consistency
- AdvantageKit replay enabled PID tuning without the robot
- State-based design prevented command sprawl

### Best Practices Reinforced

- Never let commands control motors directly
- Always log desired vs actual values
- Keep manual control paths simple and isolated

---

## Final Takeaway

This shooter architecture scales cleanly from:

- Initial bring-up
- Driver testing
- Autonomous scoring

while remaining **testable, replayable, and competition-safe**.

---

**Recommended Extensions**

- Interpolated RPM based on distance
- Shot-ready triggers
- Full superstructure integration
- Unit tests with log replay

