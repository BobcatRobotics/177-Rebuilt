package frc.robot.subsystems.Shooter;

import java.util.TreeMap;

public class ShooterTable {
    private static final TreeMap<Double, Double> speedMap = new TreeMap<Double, Double>();

    static {
        //Add values
    }

    public static double getSpeed(double distance) {
        if (speedMap.containsKey(distance)) {
            return speedMap.get(distance);
        }

        Double lowerDist = speedMap.floorKey(distance);
        Double upperDist = speedMap.ceilingKey(distance);

        if (lowerDist == null) return speedMap.get(upperDist);
        if (upperDist == null) return speedMap.get(lowerDist);

        double lowerSpeed = speedMap.get(lowerDist);
        double upperSpeed = speedMap.get(upperDist);

        double slope = (upperSpeed - lowerSpeed / upperDist - lowerDist);
        return lowerSpeed + (distance - lowerDist) * slope;

    }
}
