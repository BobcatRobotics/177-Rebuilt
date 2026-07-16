package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class TagDetail {
    private final Pose3d tagPose;
    private final int id;

    public TagDetail(Pose3d tagPose, int id) {
        this.tagPose = tagPose;
        this.id = id;
    }


    public Pose3d getPose() {
        return tagPose;
    }

    public int getId() {
        return id;
    }
    /** 3D distance */
    public double getDistance(Pose3d robotPose){
        return robotPose.getTranslation()
                        .getDistance(tagPose.getTranslation());
    }

    /** Horizontal (floor) distance only (meters) */
    public double getHorizontalDistance(Pose3d robotPose){
        Translation3d r = robotPose.getTranslation();
        Translation3d t = tagPose.getTranslation();
        return Math.hypot(r.getX() - t.getX(),
                        r.getY() - t.getY());
    }
}
