// package frc.robot.subystems.Climber;

// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.Minute;
// import static edu.wpi.first.units.Units.Rotation;
// import static edu.wpi.first.units.Units.Amps;
// import static edu.wpi.first.units.Units.Rotations;
// import static edu.wpi.first.units.Units.RotationsPerSecond;
// import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
// import static edu.wpi.first.units.Units.Seconds;
// import static edu.wpi.first.units.Units.Volts;

// import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.CANBus;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.TorqueCurrentFOC;
// import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.units.measure.AngularAcceleration;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.units.measure.Voltage;
// import frc.robot.Constants;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.units.measure.Angle;

// import frc.robot.Constants.ClimberConstants;
// import frc.robot.subystems.Climber.Modules.ModuleConfigurator;

// import org.bobcatrobotics.Util.Tunables.Gains;
// import org.bobcatrobotics.Util.Tunables.TunablePID;

// public class ClimberSim implements ClimberIO {

//   private  TalonFX climberMotor;
//   private SimMotorFX climberMotorSim;
//   public ModuleConfigurator climberConfig;

//   private final VelocityTorqueCurrentFOC requestVelocity = new VelocityTorqueCurrentFOC(0);

//   private StatusSignal<AngularVelocity> velocityOfClimberRPS;
//   private StatusSignal<Current> statorCurrentOfClimberAmps;
//   private StatusSignal<Voltage> outputOfClimberVolts;
//   private StatusSignal<AngularAcceleration> accelerationOfClimber;

//   private TorqueCurrentFOC characterizationRequestTorqueCurrentFOC = new TorqueCurrentFOC(0);
//   private VoltageOut characterizationRequestVoltage = new VoltageOut(0);


//   private TunablePID hopperTopPID;
//   private TunablePID hopperBottomPID;

//   public double hopperSetpointTop = 0;
//   public double hopperSetpointBottom = 0;

//   public ClimberSim() {

//     // Climber Config
//     Gains climberGains = new Gains.Builder()
//         .kP(Constants.ClimberConstants.kClimberP)
//         .kI(Constants.ClimberConstants.kClimberI)
//         .kD(Constants.ClimberConstants.kClimberD)
//         .kS(Constants.ClimberConstants.kClimberS)
//         .kV(Constants.ClimberConstants.kClimberV)
//         .kA(Constants.ClimberConstants.kClimberA).build();
//   }

//   public void setupTopMotor(Gains g) {
//     hopperTopPID = new TunablePID(
//         "/Hopper/Top/PID", g);
//     climberConfig = new ModuleConfigurator(g.toSlot0Configs(),
//         Constants.ClimberConstants.climberMotorId,
//         Constants.ClimberConstants.isInverted,
//         Constants.ClimberConstants.isCoast,
//         Constants.ClimberConstants.climberCurrentLimit);
//     climberMotor = new TalonFX(climberConfig.getMotorInnerId(), new CANBus("rio"));
//     climberConfig.configureMotor(climberMotor, hopperTopPID);
//     velocityOfClimberRPS = climberMotor.getVelocity();
//     statorCurrentOfClimberAmps = climberMotor.getStatorCurrent();
//     outputOfClimberVolts = climberMotor.getMotorVoltage();
//     accelerationOfClimber = climberMotor.getAcceleration();
//     climberConfig.configureSignals(climberMotor, 50.0, velocityOfClimberRPS,
//         statorCurrentOfClimberAmps, outputOfClimberVolts, accelerationOfClimber);
//   }

//   @Override
//   public void updateInputs(ClimberIOInputs inputs) {
//     climberMotorSim.update();
//     climberMotor = climberMotorSim.apply(climberMotor);

//     BaseStatusSignal.refreshAll(velocityOfClimberRPS,
//         statorCurrentOfClimberAmps, accelerationOfClimber, outputOfClimberVolts,
//         velocityOfClimberRPS,
//         statorCurrentOfClimberAmps, outputOfClimberVolts, accelerationOfClimber);

//     inputs.velocityOfClimberRPS = velocityOfClimberRPS.getValue().in(Rotation.per(Minute));
//     inputs.statorCurrentOfClimberAmps = statorCurrentOfClimberAmps.getValue().in(Amps);
//     inputs.accelerationOfClimber = accelerationOfClimber.getValue()
//         .in(RotationsPerSecondPerSecond);
//     inputs.outputOfClimberVolts = outputOfClimberVolts.getValue().in(Volts);
//     inputs.climberConnected = climberMotor.isConnected();
//   }


//      public void setPosition(ClimberState desiredState) {
//     setPosition(desiredState.getClimberPosition());
//   }

//   public void setPosition(double pos) {
//     requestPosition.withPosition(pos);
//   }


//   public void stopMotor(){
//     climberMotor.stopMotor();
//   }
//   @Override
//   public void stop() {
//     stopMotor();
//   }

//   @Override
//   public void periodic() {
//     if (climberPID.hasChanged()) {
//       climberConfig.updateMotorPID(climberMotor, climberPID);
//     }
//   }

//     /* Characterization */
//   // public void runCharacterization_Climber(double output) {
//   //   climberMotor.setControl(switch (CharacterizationClosedLoopOutputType.PositionTorqueCurrentFOC) {
//   //     case Voltage -> characterizationRequestVoltage.withOutput(output);
//   //     case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
//   //   });}

//   /** Returns the module velocity in rotations/sec (Phoenix native units). */
//   public double getFFCharacterizationVelocity_Climber() {
//         return climberMotor.getVelocity().getValue().in(RotationsPerSecond);
//   }
// 