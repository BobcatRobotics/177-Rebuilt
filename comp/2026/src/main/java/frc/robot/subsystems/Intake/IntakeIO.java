package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public class IntakeIOInputs {
    public double velocityOfIntakePositionRPS = 0.0;
    public double statorCurrentOfIntakePositionAmps = 0.0;
    public double outputOfIntakePositionVolts = 0.0;
    public double accelerationOfIntakePosition = 0.0;
    public boolean positionConnected = false;
    public double intakePosition = -1;

    public double leftVelocityOfIntakeSpeedRPS = 0.0;
    public double leftStatorCurrentOfIntakeSpeedAmps = 0.0;
    public double leftOutputOfIntakeSpeedVolts = 0.0;
    public double leftAccelerationOfIntakeSpeed = 0.0;
    public boolean leftVelocityMotorConnected = false;

    public double rightVelocityOfIntakeSpeedRPS = 0.0;
    public double rightStatorCurrentOfIntakeSpeedAmps = 0.0;
    public double rightOutputOfIntakeSpeedVolts = 0.0;
    public double rightAccelerationOfIntakeSpeed = 0.0;
    public boolean rightVelocityMotorConnected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }

  public default void manualRetractIntake(){}

 

  public default void setVelocity(double velocity) {
  }

  public default void setVelocity(IntakeState desiredState) {
  }

  public default void setPosition(IntakeState desiredState) {

  }

  public default void setPosition(double pos) {
  }

   public default void retractIntake(){}

   public default void resetEncoder() {}

  public default double getleftVelocity() {
    return 0.0;
  }

  public default double getRightVelocity() {
    return 0.0;
  }

  public default void stop() {
  }

  public default void stopLeftRollerWheel() {
  }

  public default void stopRightRollerWheel() {
  }

    public default void stopRollerWheel() {
  }


  public default void stopPivotMotor() {
  }

  public default void stopBottom() {
  }

  public default void periodic() {
  }

  public default void simulationPeriodic() {
  }

  /* Characterization */
  public default void runCharacterization_IntakeVelocity(double output) {
  }

  /* Characterization */
  public default void runCharacterization_IntakePosition(double output) {
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public default double getFFCharacterizationVelocity_Intake() {
    return 0.0;
  }

  /** Returns the module position. */
  public default double getFFCharacterizationPosition_Intake() {
    return 0.0;
  }

  public default void holdPosition() {

  }

  public default void setNeturalCoast(){

  }
  public default void setNeturalBrake(){
    
  }
}