package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class DistanceUtility {
    public double actualDistance = 0.0;
    public double offsetDistance = 0.0;
    public DistanceUtility(double distanceInMeters, double offset){
        actualDistance = Units.metersToInches(distanceInMeters);
        offsetDistance = actualDistance - Units.metersToInches(offset);
    }
    public double getActualDistance(){
        return actualDistance;
    }
    public double getOffsetDistance(){
        return offsetDistance;
    }
}