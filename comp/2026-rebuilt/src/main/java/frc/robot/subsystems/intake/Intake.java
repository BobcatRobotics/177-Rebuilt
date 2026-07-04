package frc.robot.subsystems.intake;

import org.bobcatrobotics.Hardware.Characterization.SysIdModule;
import org.bobcatrobotics.Hardware.Characterization.SysIdRegistry;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Intake/inputs", inputs);
    }

    public void stop() {
        io.stop();
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }
}
