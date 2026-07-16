package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GamesirG7 extends CommandXboxController {
    private CommandJoystick controller;
    public Trigger leftPaddle;
    public Trigger rightPaddle;

    public GamesirG7(int port) {
        super(port);
        controller = new CommandJoystick(port);
        leftPaddle = super.button(9);
        rightPaddle = super.button(10);
    }

    public Trigger getLeftPaddle() {
        return leftPaddle;
    }

    public Trigger getRightPaddle() {
        return rightPaddle;
    }

}
