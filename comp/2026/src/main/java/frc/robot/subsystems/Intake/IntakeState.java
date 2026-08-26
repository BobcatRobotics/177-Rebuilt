package frc.robot.subsystems.Intake;
    public enum IntakeState{
        IDLE(0),
        ROLLING_IN(30),
        ROLLING_OUT(-30),
        INTAKE_IN(0),
        INTAKE_OUT(0);
        
        private final double rps;

        IntakeState(double rps){
            this.rps = rps;
        }

    public double getRPS() {
        return rps;
    }


}