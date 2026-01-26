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
    public double kickerVelocityRPM = 0.0;

    public double topAppliedVolts = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double kickerAppliedVolts = 0.0;

    public boolean topConnected = false;
    public boolean bottomConnected = false;
    public boolean kickerConnected = false;
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
  private final TalonFX kickerMotor = new TalonFX(12);

  private final VelocityTorqueCurrentFOC requestTop = new VelocityTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC requestBottom = new VelocityTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC requestKicker = new VelocityTorqueCurrentFOC(0);

    private StatusSignal<Double> topMotorStatorCurrent;
    private StatusSignal<Double> topMotorVelocityRPS;
    private StatusSignal<Double> bottomMotorStatorCurrent;
    private StatusSignal<Double> bottomMotorVelocityRPS;
    private StatusSignal<Double> kickerMotorStatorCurrent;
    private StatusSignal<Double> kickerMotorVelocityRPS;

  public ShooterReal() {
    // Top motor configurations
    TalonFXConfiguration topConfigs = new TalonFXConfiguration();
    topMotor.getConfigurator().apply(topConfigs); // reset to default
    topConfigs.MotorOutput.Inverted = ShooterConstants.topMotorInvert;
    topConfigs.MotorOutput.NeutralMode = ShooterConstants.topMotorBrakeMode;
    topConfigs.Slot0.kP = ShooterConstants.kTopP;
    topConfigs.Slot0.kV = ShooterConstants.kTopV;
    topConfigs.Slot0.kS = ShooterConstants.kTopS;
    topConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    topConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.topCurrentLimit;
    topMotor.getConfigurator().apply(topConfigs);
    // Bottom motor configurations
    TalonFXConfiguration bottomConfigs = new TalonFXConfiguration();
    bottomMotor.getConfigurator().apply(bottomConfigs); // reset to default
    bottomConfigs.MotorOutput.Inverted = ShooterConstants.bottomMotorInvert;
    bottomConfigs.MotorOutput.NeutralMode = ShooterConstants.bottomMotorBrakeMode;
    bottomConfigs.Slot0.kP = ShooterConstants.kBottomP;
    bottomConfigs.Slot0.kV = ShooterConstants.kBottomV;
    bottomConfigs.Slot0.kS = ShooterConstants.kBottomS;
    bottomConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    bottomConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.bottomCurrentLimit;
    bottomMotor.getConfigurator().apply(bottomConfigs);
    // Kicker motor configurations
    TalonFXConfiguration kickerConfigs = new TalonFXConfiguration();
    bottomMotor.getConfigurator().apply(kickerConfigs); // reset to default
    kickerConfigs.MotorOutput.Inverted = ShooterConstants.kickerMotorInvert;
    kickerConfigs.MotorOutput.NeutralMode = ShooterConstants.kickerMotorBrakeMode;
    kickerConfigs.Slot0.kP = ShooterConstants.kKickerP;
    kickerConfigs.Slot0.kV = ShooterConstants.kKickerV;
    kickerConfigs.Slot0.kS = ShooterConstants.kKickerS;
    kickerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    kickerConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.kickerCurrentLimit;
    kickerMotor.getConfigurator().apply(kickerConfigs);
    // Apply to signals
    topMotorStatorCurrent = topMotor.getStatorCurrent();
    topMotorVelocityRPS = topMotor.getVelocity();
    bottomMotorStatorCurrent = bottomMotor.getStatorCurrent();
    bottomMotorVelocityRPS = bottomMotor.getVelocity();
    kickerMotorStatorCurrent = kickerMotor.getStatorCurrent();
    kickerMotorVelocityRPS = kickerMotor.getVelocity();
    // Set polling frequency and optimizations
    BaseStatusSignal.setUpdateFrequencyForAll(50, topMotorStatorCurrent, topMotorVelocityRPS, bottomMotorStatorCurrent, bottomMotorVelocityRPS,kickerMotorStatorCurrent, kickerMotorVelocityRPS);
    topMotor.optimizeBusUtilization();
    bottomMotor.optimizeBusUtilization();
    kickerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        topMotorStatorCurrent,
        topMotorVelocityRPS,
        bottomMotorStatorCurrent,
        bottomMotorVelocityRPS,
        kickerMotorStatorCurrent,
        kickerMotorVelocityRPS);

    inputs.topVelocityRPM = topMotor.getVelocity().getValue() * 60.0;
    inputs.bottomVelocityRPM = bottomMotor.getVelocity().getValue() * 60.0;
    inputs.kickerVelocityRPM = kickerMotor.getVelocity().getValue() * 60.0;

    inputs.topAppliedVolts = topMotor.getMotorVoltage().getValue();
    inputs.bottomAppliedVolts = bottomMotor.getMotorVoltage().getValue();
    inputs.kickerAppliedVolts = kickerMotor.getMotorVoltage().getValue();

    inputs.topConnected = topMotor.isConnected();
    inputs.bottomConnected = bottomMotor.isConnected();
    inputs.kickerConnected = kickerMotor.isConnected();
  }

  @Override
  public void setTopVelocity(double rpm) {
    topMotor.setControl(velocityTopRequest.withVelocity(rpm / 60.0));
  }

  @Override
  public void setBottomVelocity(double rpm) {
    bottomMotor.setControl(velocityBottomRequest.withVelocity(rpm / 60.0));
  }

  @Override
  public void setKickerVelocity(double rpm) {
    kickerMotor.setControl(velocityKickerRequest.withVelocity(rpm / 60.0));
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
    kickerMotor.stopMotor();
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

  private final FlywheelSim kickerSim =
      new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.025);

  private double topVolts = 0.0;
  private double bottomVolts = 0.0;
  private double kickerVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    topSim.update(0.02);
    bottomSim.update(0.02);
    kickerSim.update(0.02);

    inputs.topVelocityRPM = topSim.getAngularVelocityRPM();
    inputs.bottomVelocityRPM = topSim.getAngularVelocityRPM();
    inputs.kickerVelocityRPM = topSim.getAngularVelocityRPM();

    inputs.topAppliedVolts = topVolts;
    inputs.bottomAppliedVolts = bottomVolts;
    inputs.kickerAppliedVolts = kickerVolts;

    inputs.topConnected = true;
    inputs.bottomConnected = true;
    inputs.kickerConnected = true;
  }

  @Override
  public void setTopVoltage(double volts) {
    topVolts = volts;
    topSim.setInputVoltage(volts);
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomVolts = volts;
    bottomSim.setInputVoltage(volts);
  }

  @Override
  public void setKickerVoltage(double volts) {
    kickerVolts = volts;
    kickerSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setTopVoltage(0);
    setBottomVoltage(0);
    setKickerVoltage(0);
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
  private double manualKickerVolts = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/State", desiredState.name());
    Logger.recordOutput("Shooter/ManualMode", manualMode);

    if (manualMode) {
      io.setTopVoltage(manualTopVolts);
      io.setBottomVoltage(manualBottomVolts);
      io.setKickerVoltage(manualKickerVolts);
    } else {
      io.setTopVelocity(desiredState.topRPM);
      io.setBottomVelocity(desiredState.bottomRPM);
      io.setKickerVelocity(desiredState.kickerRPM);
    }
  }

  public void setManualVoltage(double top, double bottom, double kicker) {
        manualMode = true;
    manualTopVolts = top;
    manualBottomVolts = bottom;
    manualKickerVolts = kicker;
  }

  public void exitManualMode() {
     manualMode = false;
  }

  public void setState(ShooterState state) {
    manualMode = false;
    desiredState = state;
  }

  public boolean atSetpoint() {
    return Math.abs(inputs.topVelocityRPM - desiredState.topRPM) < 100
        && Math.abs(inputs.bottomVelocityRPM - desiredState.bottomRPM) < 100 && Math.abs(inputs.kickerVelocityRPM - desiredState.kickerRPM) < 100;
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

