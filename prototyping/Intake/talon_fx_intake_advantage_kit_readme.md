# Dual TalonFX Intake – WPILib + AdvantageKit

## Development Story

This document describes the design, architecture, and implementation of a **dual-motor intake subsystem** using **two independently controlled TalonFX (Falcon / Kraken)** motors.  
The system is implemented using **WPILib**, **CTRE Phoenix 6**, and **AdvantageKit**, and supports **manual control** and **command-based control** with full simulation support.

---

## 1. Problem Definition & Mechanical Reality

Early in the season, the team identified that the intake needed to:

- Reliably acquire game pieces from imperfect angles
- Handle off-center contact
- Recover quickly from jams

To accomplish this, the intake uses **two independently driven rollers**:

- Intake motor – roller

This allows asymmetric control for centering and unjamming.

---

### Design Requirements

- Independent control of both motors
- Manual open-loop control for bring-up and pit use
- Command-based control for match play
- AdvantageKit IO abstraction
- Identical behavior in sim and real

---

## 2. Architecture Overview

The intake follows the standard **AdvantageKit IO abstraction pattern**.

```
Intake (Subsystem)
 ├── IntakeIO (interface)
 │    ├── IntakeIOInputs (AutoLogged)
 │
 ├── IntakeReal (TalonFX hardware)
 ├── IntakeSim  (FlywheelSim-based)
 │
 └── IntakeState (desired intake behavior)
```

### Architectural Intent

| Component | Responsibility |
|---------|---------------|
| `Intake` | Control logic, state management, logging |
| `IntakeIO` | Hardware abstraction contract |
| `IntakeReal` | CTRE TalonFX implementation |
| `IntakeSim` | Physics-based flywheel simulation |
| `IntakeState` | High-level intake behavior definition |

---

---

## 3. IntakeState – Defining Intent

Rather than commands directly controlling motors, the intake operates on **high-level states**.

```java
public enum IntakeState {
  OFF(0.0),
  FORWARD(8.0),
  REVERSE(-8.0);

  public final double leftVolts;

  IntakeState(double leftVolts) {
    this.leftVolts = leftVolts;
  }
}
```

### Benefits

- Centralized tuning
- Easy logging & replay
- Clean command-based API
- Prevents motor control logic from spreading into commands

---

## 4. IntakeIO – Hardware Contract

```java
public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public double intakeVelocityRPM = 0.0;

    public double intakeAppliedVolts = 0.0;

    public boolean intakeConnected = false;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setIntakeVoltage(double volts) {}

  default void stop() {}
}
```

---

## 5. IntakeReal – TalonFX Implementation

```java
public class IntakeReal implements IntakeIO {

  private final TalonFX intakeMotor = new TalonFX(30);
  private final VelocityRequest requestTop = new VelocityRequest(0);
  private StatusSignal<Double> intakeMotorStatorCurrent;
  private StatusSignal<Double> intakeMotorVelocityRPS;
  public IntakeReal() {
    // Top motor configurations
    TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
    intakeMotor.getConfigurator().apply(intakeConfigs); // reset to default
    intakeConfigs.MotorOutput.Inverted = IntakeConstants.intakeMotorInvert;
    intakeConfigs.MotorOutput.NeutralMode = IntakeConstants.intakeMotorBrakeMode;
    intakeConfigs.Slot0.kP = IntakeConstants.kTopP;
    intakeConfigs.Slot0.kV = IntakeConstants.kTopV;
    intakeConfigs.Slot0.kS = IntakeConstants.kTopS;
    intakeConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.topCurrentLimit;
    topMotor.getConfigurator().apply(intakeConfigs);
    // Apply to signals
    intakeMotorStatorCurrent = topMotor.getStatorCurrent();
    intakeMotorVelocityRPS = topMotor.getVelocity();
    // Set polling frequency and optimizations
    BaseStatusSignal.setUpdateFrequencyForAll(50, intakeMotorStatorCurrent, intakeMotorVelocityRPS);
    intakeMotor.optimizeBusUtilization();

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelocityRPM = intakeMotor.getVelocity().getValue() * 60.0;
    inputs.intakeAppliedVolts = intakeMotor.getMotorVoltage().getValue();
    inputs.intakeConnected = intakeMotor.isConnected();
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
  }
}
```

---

## 6. IntakeSim – Simulation

```java
public class IntakeSim implements IntakeIO {

  private final FlywheelSim intakeSim =
      new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.01);

  private double intakeVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.update(0.02);
    inputs.intakeVelocityRPM = intakeSim.getAngularVelocityRPM();
    inputs.intakeAppliedVolts = intakeVolts;
    inputs.intakeConnected = true;
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeVolts = volts;
    intakeSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setIntakeVoltage(0);
  }
}
```

---

## 7. Intake Subsystem – Core Logic

The `Intake` subsystem:

- Owns the current `IntakeState`
- Arbitrates manual vs command-based control
- Logs all inputs and outputs
- Remains hardware-agnostic

```java
public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs =
      new IntakeIOInputsAutoLogged();

  private IntakeState desiredState = IntakeState.OFF;

  private boolean manualMode = false;
  private double manualIntakeVolts = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/State", desiredState.name());
    Logger.recordOutput("Intake/ManualMode", manualMode);

    if (manualMode) {
      io.setLeftVoltage(manualIntakeVolts);
    } else {
      io.setLeftVoltage(desiredState.intakeVolts);
    }
  }

  public void setManualVoltage(double intakeVoltage) {
    manualMode = true;
    manualIntakeVolts = intakeVoltage;
  }

  public void setState(IntakeState state) {
    manualMode = false;
    desiredState = state;
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
    () -> intake.setIntakeVoltage(
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
  public Command setStateCommand(IntakeState state) {
    return this.runOnce(() -> setState(state))
        .andThen(
            Commands.waitUntil(this::atSetpoint))
        .withName("Intake:SetState:" + state.name());
  }

  /**
   * Continuously holds a shooter state.
   * Useful for parallel or default commands.
   */
  public Command holdStateCommand(IntakeState state) {
    return this.run(() -> setState(state))
        .withName("Intake:HoldState:" + state.name());
  }
```

Usage example:

```java
new SetIntakeState(intake, IntakeState.FORWARD)
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

This dual-motor intake design supports:

- Independent motor control
- Manual bring-up and recovery
- Clean command-based composition
- Full AdvantageKit logging and replay
- Simulation-first development

Even “simple” mechanisms deserve robust architecture.
