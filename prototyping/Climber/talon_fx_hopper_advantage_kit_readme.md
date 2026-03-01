# TalonFX Hopper – WPILib + AdvantageKit

This document describes the **design, architecture, and implementation** of a **robot hopper subsystem** driven by a **single TalonFX (Falcon / Kraken)** motor.  
The hopper is responsible for **rotational aiming** of the shooter and supports **manual control**, **command-based closed-loop control**, and **full simulation**, all implemented using **WPILib**, **CTRE Phoenix 6**, and **AdvantageKit**.

---

## 1. Problem Definition & Goals

The hopper provides precise angular positioning to align the shooter with field targets. Unlike drivetrain mechanisms, the hopper:

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

The hopper follows the standard **AdvantageKit IO abstraction pattern**, mirroring other subsystems such as the shooter.

```
Hopper (Subsystem)
 ├── HopperIO (interface)
 │    ├── HopperIOInputs (AutoLogged)
 │
 ├── HopperReal (TalonFX hardware)
 ├── HopperSim  (SingleJointedArmSim-based)
 │
 └── HopperState (desired hopper behavior)
```

### Architectural Intent

| Component | Responsibility |
|---------|---------------|
| `Hopper` | Control logic, limits, logging |
| `HopperIO` | Hardware abstraction contract |
| `HopperReal` | CTRE TalonFX position control |
| `HopperSim` | Physics-based angular simulation |
| `HopperState` | High-level hopper intent |

---

## 3. HopperState – Defining Intent

The hopper operates on **high-level intent states**, rather than raw motor commands.

```java
public enum HopperState {
  IDLE(0.0),
  ON(0.0),
  OFF(45.0);

  public final double angleDeg;

  HopperState(double angleDeg) {
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

## 4. HopperIO – Hardware Contract

The IO interface defines the **exact data and controls** the subsystem requires.

```java
public interface HopperIO {

  @AutoLog
  class HopperIOInputs {
    public double positionDeg = 0.0;
    public double velocityDegPerSec = 0.0;
    public double statorCurrent = 0.0;

    public double appliedVolts = 0.0;
    public boolean motorConnected = false;
  }

  default void updateInputs(HopperIOInputs inputs) {
  }

  default void setVoltage(double volts) {}
  default void setPosition(double angleDeg) {}

  default void stop() {}
}
```

---

## 5. HopperReal – TalonFX Implementation

The real hopper implementation uses **CTRE Phoenix 6 position control**.

```java
public class HopperReal implements HopperIO {

  private final TalonFX hopperMotor = new TalonFX(20);
  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private StatusSignal<Double> hopperMotorStatorCurrent;
  private StatusSignal<Double> hopperMotorVelocityRPS;
  private StatusSignal<Double> hopperMotorPositionDegrees;
  private StatusSignal<Double> hopperMotorVolts;

  public HopperReal() {
    // Top motor configurations
    TalonFXConfiguration hopperConfigs = new TalonFXConfiguration();
    hopperMotor.getConfigurator().apply(hopperConfigs); // reset to default
    hopperConfigs.MotorOutput.Inverted = HopperConstants.intakeMotorInvert;
    hopperConfigs.MotorOutput.NeutralMode = HopperConstants.intakeMotorBrakeMode;
    hopperConfigs.Slot0.kP = HopperConstants.kTopP;
    hopperConfigs.Slot0.kV = HopperConstants.kTopV;
    hopperConfigs.Slot0.kS = HopperConstants.kTopS;
    hopperConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    hopperConfigs.CurrentLimits.StatorCurrentLimit = HopperConstants.topCurrentLimit;
    topMotor.getConfigurator().apply(hopperConfigs);
    // Apply to signals
    hopperMotorStatorCurrent = topMotor.getStatorCurrent();
    hopperMotorPositionDegrees = topMotor.getVelocity().getValue() * 360.0;
    hopperMotorVelocityRPS = topMotor.getStatorCurrent();
    hopperMotorVolts = topMotor.getMotorVoltage();
    // Set polling frequency and optimizations
    BaseStatusSignal.setUpdateFrequencyForAll(50, hopperMotorStatorCurrent, hopperMotorPositionDegrees);
    hopperMotor.optimizeBusUtilization();
  }



  @Override
  public void updateInputs(HopperIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        hopperMotorStatorCurrent,
    hopperMotorVolts,
        hopperMotorPositionDegrees,hopperMotorVelocityRPS);

    inputs.positionDeg = hopperMotorPositionDegrees.getValueAsDouble()* 360.0;
    inputs.velocityDegPerSec = hopperMotorVelocityRPS.getValueAsDouble() * 360.0;
    inputs.statorCurrent = hopperMotorStatorCurrent.getValueAsDouble();
    inputs.appliedVolts = hopperMotorVolts.getValueAsDouble();;
    inputs.motorConnected = motor.isConnected();
  }

  @Override
  public void setPosition(double angleDeg) {
    hopperMotor.setControl(positionRequest.withPosition(angleDeg / 360.0));
  }

  @Override
  public void stop() {
    hopperMotor.stopMotor();
  }
}
```

---

## 6. HopperSim – Angular Simulation

Simulation uses WPILib’s `SingleJointedArmSim`, which maps well to hopper dynamics.

```java
public class HopperSim implements HopperIO {

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
  public void updateInputs(HopperIOInputs inputs) {
    sim.update(0.02);
    inputs.positionDeg = Units.radiansToDegrees(sim.getAngleRads());
    inputs.velocityDegPerSec = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.statorCurrent = 0;
    inputs.appliedVolts = appliedVolts;
    inputs.motorConnected = true;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }
}
```

---

## 7. Hopper Subsystem – Core Logic

The subsystem arbitrates between **manual control** and **state-based control**, while enforcing limits and logging.

```java
public class Hopper extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs =
      new HopperIOInputsAutoLogged();

  private HopperState desiredState = HopperState.IDLE;
  private boolean manualMode = false;
  private double manualVolts = 0.0;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    Logger.recordOutput("Hopper/State", desiredState.name());
    Logger.recordOutput("Hopper/ManualMode", manualMode);

    if (manualMode) {
      io.setVoltage(manualVolts);
    } else {
      io.setPosition(desiredState.angleDeg);
    }
  }

  public void setManualVoltage(double volts) {
        manualMode = true;
    manualVolts = volts;
  }

  public void exitManualMode() {
      manualMode = false;
  }

  public void setState(HopperState state) {
        manualMode = false;
    desiredState = state;
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
    () -> hopper.setManualVoltage(-driver.getRightX() * 6.0),
    hopper)
```

---

## 9. Command-Based Hopper Control

Commands request **intent**, not voltage.

```java

  /**
   * Command that sets the hopper to a desired state and finishes
   * once the hopper reaches its setpoint.
   */
  public Command setStateCommand(HopperState state) {
    return this.runOnce(() -> setState(state))
        .andThen(
            Commands.waitUntil(this::atSetpoint))
        .withName("Hopper:SetState:" + state.name());
  }

  /**
   * Command that continuously holds a hopper state
   * (useful for default or parallel commands).
   */
  public Command holdStateCommand(HopperState state) {
    return this.run(() -> setState(state))
        .withName("Hopper:HoldState:" + state.name());
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

This hopper architecture cleanly supports:

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

