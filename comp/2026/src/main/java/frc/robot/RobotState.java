package frc.robot;

import frc.robot.subsystems.Carwash.Carwash;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class RobotState {
    private final Hopper hopper; 
    private final Shooter shooter;
    private final Intake intake;
    private final Carwash carwash;
    private final Hood hood;

    //private RobotState state = //RobotState.IDLE;

    RobotState(Hopper hopper, Shooter shooter, Intake intake, Carwash carwash, Hood hood){
      this.hopper = hopper;
      this.shooter = shooter;
      this.intake = intake;
      this.carwash = carwash;
      this.hood = hood; 
    }
}
