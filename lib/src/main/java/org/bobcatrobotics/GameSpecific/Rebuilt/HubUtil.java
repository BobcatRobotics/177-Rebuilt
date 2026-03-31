package org.bobcatrobotics.GameSpecific.Rebuilt;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Inches;

import java.io.IOException;
import java.util.Optional;

/**
 * Minimal helper to determine hub ownership and remaining active time.
 * Automatically pulses alerts when ownership changes.
 */
public final class HubUtil {
    private static final double HUB_ACTIVE_START = 15.0;
    private static final double HUB_ACTIVE_END = 135.0;
    private static final double ALERT_PULSE_DURATION = 2.0;

    private static final Alert RED_ALERT = new Alert("Red has taken ownership!", AlertType.kWarning);
    private static final Alert BLUE_ALERT = new Alert("Blue has taken ownership!", AlertType.kWarning);

    private static double redEndTime = 0;
    private static double blueEndTime = 0;

    public HubUtil() {
    }

    public static HubData getHubData() {
        double matchTime = DriverStation.getMatchTime();
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        String gameData = DriverStation.getGameSpecificMessage();

        // Inactive or missing info
        if (alliance.isEmpty() || gameData == null || gameData.isEmpty()
                || matchTime < HUB_ACTIVE_START || matchTime > HUB_ACTIVE_END) {
            RED_ALERT.set(false);
            BLUE_ALERT.set(false);
            return new HubData(HubOwner.NONE, 0.0);
        }

        HubOwner owner = switch (gameData.charAt(0)) {
            case 'R' -> HubOwner.RED;
            case 'B' -> HubOwner.BLUE;
            default -> HubOwner.NONE;
        };

        double now = Timer.getFPGATimestamp();
        // Trigger pulse if alert not already active
        if (owner == HubOwner.RED && !RED_ALERT.get()) {
            RED_ALERT.set(true);
            redEndTime = now + ALERT_PULSE_DURATION;
            BLUE_ALERT.set(false);
        } else if (owner == HubOwner.BLUE && !BLUE_ALERT.get()) {
            BLUE_ALERT.set(true);
            blueEndTime = now + ALERT_PULSE_DURATION;
            RED_ALERT.set(false);
        }

        // Auto-off after pulse
        if (RED_ALERT.get() && now >= redEndTime) {
            RED_ALERT.set(false);
        }
        if (BLUE_ALERT.get() && now >= blueEndTime) {
            BLUE_ALERT.set(false);
        }

        double timeRemaining = owner != HubOwner.NONE ? Math.max(0.0, HUB_ACTIVE_END - matchTime) : 0.0;
        return new HubData(owner, timeRemaining);
    }

    /**
     * Returns the field-relative {@link Pose3d} of the active Hub target based on
     * the robot's alliance color and the current hub ownership.
     *
     * <p>
     * This method assumes a standard WPILib field coordinate system:
     * +X runs from the Blue alliance wall toward the Red alliance wall.
     * The Blue-side hub is defined at (4.620, 4.040, 3.057144).
     * The Red-side hub is computed by mirroring across the field length
     * and rotating 180 degrees about the Z-axis.
     * </p>
     *
     * <p>
     * If the hub is owned by the same alliance as the robot, the method
     * returns the hub on the robot's alliance side. Otherwise, it returns
     * the opposing alliance hub.
     * </p>
     *
     * @param alliance The current {@code Alliance} of the robot (from
     *                 DriverStation).
     * @return The {@code Pose3d} of the hub the robot should target.
     */
    public static Pose3d getActiveHubCoordinates(Alliance alliance) {
        Pose3d blueHub = new Pose3d(
                    hubPosition(Alliance.Blue).getX(),
                    hubPosition(Alliance.Blue).getY(),
                    1.12395,
                    new Rotation3d());
        Pose3d redHub = new Pose3d(
                    hubPosition(Alliance.Red).getX(),
                    hubPosition(Alliance.Red).getY(),
                    1.12395,
                    new Rotation3d());

        HubData hub = getHubData();

        boolean sameOwner = (alliance == Alliance.Red && hub.owner == HubOwner.RED) ||
                (alliance == Alliance.Blue && hub.owner == HubOwner.BLUE);

        Pose3d currentAllianceHub  = new Pose3d();
        Pose3d opposingAllianceHub = new Pose3d();
        if (alliance == Alliance.Red) {
            if (sameOwner) {
                currentAllianceHub = redHub;
                opposingAllianceHub = blueHub;
            } else {
                currentAllianceHub = blueHub;
                opposingAllianceHub = redHub;
            }
        } else {
            if (sameOwner) {
                currentAllianceHub = blueHub;
                opposingAllianceHub = redHub;
            } else {
                currentAllianceHub = redHub;
                opposingAllianceHub = blueHub;
            }
        }
               
        return sameOwner ? currentAllianceHub : opposingAllianceHub;
    }

    /**
     * Returns the field-relative {@link Pose3d} of my Hub target based on
     * the robot's alliance color
     *
     * <p>
     * This method assumes a standard WPILib field coordinate system:
     * +X runs from the Blue alliance wall toward the Red alliance wall.
     * The Blue-side hub is defined at (4.620, 4.040, 3.057144).
     * The Red-side hub is computed by mirroring across the field length
     * and rotating 180 degrees about the Z-axis.
     * </p>
     *
     * <p>
     * If the hub is owned by the same alliance as the robot, the method
     * returns the hub on the robot's alliance side. Otherwise, it returns
     * the opposing alliance hub.
     * </p>
     *
     * @param alliance The current {@code Alliance} of the robot (from
     *                 DriverStation).
     * @return The {@code Pose3d} of the hub the robot should target.
     */
    public static Pose3d getMyHubCoordinates(Alliance alliance) {
        Pose3d hubPose = new Pose3d();
        Translation2d hub = hubPosition(alliance);
        hubPose = new Pose3d(
                    hub.getX(),
                    hub.getY(),
                    1.12395,
                    new Rotation3d());
        return hubPose;
    }

    public static Translation2d hubPosition(Alliance allianceGet ) {
        if (allianceGet == Alliance.Blue) {
            return new Translation2d(Units.inchesToMeters(182.105), Units.inchesToMeters(158.845));
        }
        return new Translation2d(Units.inchesToMeters(469.115), Units.inchesToMeters(158.845));
    }

    
}