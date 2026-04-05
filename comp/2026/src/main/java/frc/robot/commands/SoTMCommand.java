package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.bobcatrobotics.Util.Interpolators.TripleOutputInterpolator;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.Carwash.Carwash;
import frc.robot.subsystems.Carwash.CarwashState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drive.Drive;

public class SoTMCommand extends Command{
    private final Shooter shooter;
    private final Drive drive;
    private final Carwash carwash;

    public SoTMCommand(Shooter shooter, Drive drive, Carwash carwash) {
        this.shooter = shooter;
        this.drive = drive;
        this.carwash = carwash;

        addRequirements(shooter, carwash);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds robotSpeed = drive.getChassisSpeeds();

    //Get calculated speeds
    TripleOutputInterpolator.Speeds speeds = ShootOnTheMove.calculateSpeeds(drive, robotSpeed);
    shooter.setVelocity(speeds.one, speeds.three, speeds.three);
    carwash.setVelocity(CarwashState.State.INTERPOLATING);
    
    Logger.recordOutput("ShootOnTheMove/mainFlyWheelSpeed", speeds.one);
    Logger.recordOutput("ShootOnTheMove/hoodFlyWheelSpeed", speeds.three);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
