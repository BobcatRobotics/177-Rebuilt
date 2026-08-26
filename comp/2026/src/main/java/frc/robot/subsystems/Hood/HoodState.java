package frc.robot.subsystems.Hood;

public enum HoodState {
    IDLE(0.0),
    ANGLED(35);

    private final double angle;
    HoodState(double angle){
        this.angle = angle;
    }

    public double getAngle(){
        return this.angle;
    }
}
