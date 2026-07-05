package frc.robot.subsystems.hopper;

public enum HopperState {
    IDLE(0.0),
    SPINUP(-30),
    INTAKE(60),
    OUTAKE(-50);
    
    private final double rollerSpeed;

    HopperState(double velocity){
        this.rollerSpeed = velocity;
    }
    public double getRollerSpeed() {
        return rollerSpeed;
    }
}