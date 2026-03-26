package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.util.Units;

public class Distance {
    public double actualDistance = 0.0;
    public double offsetDistance = 0.0;
    public Distance(double distanceInMeters, double offset){
        actualDistance = distanceInMeters;
        offsetDistance = actualDistance - offset;
    }
    public double getActualDistance(){
        return actualDistance;
    }
    public double getOffsetDistance(){
        return offsetDistance;
    }
}