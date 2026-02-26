package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TripleSpeedInterpolator;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drive.Drive;

public class SoTMCommand extends Command{
    private final Shooter shooter;
    private final Drive drive;

    public SoTMCommand(Shooter shooter, Drive drive) {
        this.shooter = shooter;
        this.drive = drive;

        addRequirements(shooter, drive);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds robotSpeed = drive.getChassisSpeeds();

    // Check if shot is valid
    if (!ShootOnTheMove.isShotValid(robotPose)) {
      shooter.stop(); // when outside bounds
      return;
    }

    // Get calculated speeds
    TripleSpeedInterpolator.Speeds speeds = ShootOnTheMove.calculateRequiredSpeeds(robotPose, robotSpeed);
    shooter.setVelocity(speeds.one, speeds.two, speeds.three); 
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
