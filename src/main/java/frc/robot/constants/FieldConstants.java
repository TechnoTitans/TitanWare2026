package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Set;

public class FieldConstants {
    public static final double FIELD_LENGTH_X_METERS = Units.inchesToMeters(651.22);
    public static final double FIELD_WIDTH_Y_METERS = Units.inchesToMeters(317.69);
    public static final Pose2d RED_ORIGIN = new Pose2d(FIELD_LENGTH_X_METERS, FIELD_WIDTH_Y_METERS, Rotation2d.k180deg);

    public static final Pose2d BLUE_HUB_POSE = new Pose2d(
            4.626,
            4.033,
            Rotation2d.kZero
    );

    public static final Pose2d RED_HUB_POSE = BLUE_HUB_POSE.relativeTo(RED_ORIGIN);

    public static final double BLUE_FERRY_X_BOUNDARY = BLUE_HUB_POSE.getX();
    public static final double RED_FERRY_X_BOUNDARY = RED_HUB_POSE.getX();

    public static final double BLUE_FERRY_LEFT_Y_BOUNDARY = (FIELD_WIDTH_Y_METERS / 2) - Units.inchesToMeters(12);
    public static final double BLUE_FERRY_RIGHT_Y_BOUNDARY = (FIELD_WIDTH_Y_METERS / 2) + Units.inchesToMeters(12);

    public static final double RED_FERRY_LEFT_Y_BOUNDARY = BLUE_FERRY_RIGHT_Y_BOUNDARY;
    public static final double RED_FERRY_RIGHT_Y_BOUNDARY = BLUE_FERRY_LEFT_Y_BOUNDARY;

    public static final Pose2d BLUE_FERRY_LEFT = new Pose2d(4, 2, Rotation2d.kZero);
    public static final Pose2d BLUE_FERRY_RIGHT = new Pose2d(4, 6, Rotation2d.kZero);

    public static final Pose2d RED_FERRY_LEFT = BLUE_FERRY_LEFT.relativeTo(RED_ORIGIN);
    public static final Pose2d RED_FERRY_RIGHT = BLUE_FERRY_RIGHT.relativeTo(RED_ORIGIN);

    private static final double TRENCH_WIDTH = Units.inchesToMeters(6);
    private static final double TURRET_SAFE_BUFFER = Units.inchesToMeters(12);
    private static final double TURRET_MOVING_CLOSE_SAFE_BUFFER = Units.inchesToMeters(12);
    private static final double TURRET_MOVING_FAR_SAFE_BUFFER = Units.inchesToMeters(36);
    public static final double BLUE_TURRET_SAFE_X_CLOSE_BOUNDARY = BLUE_HUB_POSE.getX()
            - (TRENCH_WIDTH / 2)
            - TURRET_SAFE_BUFFER;
    public static final double RED_TURRET_SAFE_X_CLOSE_BOUNDARY = RED_HUB_POSE.getX()
            + (TRENCH_WIDTH / 2)
            + TURRET_SAFE_BUFFER;

    public static final double BLUE_TURRET_SAFE_X_FAR_BOUNDARY = BLUE_HUB_POSE.getX()
            + (TRENCH_WIDTH / 2)
            + TURRET_SAFE_BUFFER;
    public static final double RED_TURRET_SAFE_X_FAR_BOUNDARY = RED_HUB_POSE.getX()
            - (TRENCH_WIDTH / 2)
            - TURRET_SAFE_BUFFER;

    public static final Set<Integer> BLUE_APRILTAG_IDS =
            Set.of(17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);

    public static final Set<Integer> RED_APRILTAG_IDS =
            Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);

    private static <T> T getAllianceFlipped(final T blueAlliance, final T redAlliance) {
        return DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Red ? redAlliance : blueAlliance)
                .orElse(blueAlliance);
    }

    public static Pose2d getHubPose() {
        return getAllianceFlipped(BLUE_HUB_POSE, RED_HUB_POSE);
    }

    public static double getFerryXBoundary() {
        return getAllianceFlipped(BLUE_FERRY_X_BOUNDARY, RED_FERRY_X_BOUNDARY);
    }

    public static double getFerryLeftYBoundary() {
        return getAllianceFlipped(BLUE_FERRY_LEFT_Y_BOUNDARY, RED_FERRY_LEFT_Y_BOUNDARY);
    }

    public static double getFerryRightYBoundary() {
        return getAllianceFlipped(BLUE_FERRY_RIGHT_Y_BOUNDARY, RED_FERRY_RIGHT_Y_BOUNDARY);
    }

    public static Pose2d getFerryLeft() {
        return getAllianceFlipped(BLUE_FERRY_LEFT, RED_FERRY_LEFT);
    }

    public static Pose2d getFerryRight() {
        return getAllianceFlipped(BLUE_FERRY_RIGHT, RED_FERRY_RIGHT);
    }

    public static double getTurretSafeXCloseBoundary() {
        return getAllianceFlipped(BLUE_TURRET_SAFE_X_CLOSE_BOUNDARY, RED_TURRET_SAFE_X_CLOSE_BOUNDARY);
    }

    public static double getTurretSafeXFarBoundary() {
        return getAllianceFlipped(BLUE_TURRET_SAFE_X_FAR_BOUNDARY, RED_TURRET_SAFE_X_FAR_BOUNDARY);
    }

    public static double getMovingTurretSafeXCloseBoundary() {
        return getAllianceFlipped(
                BLUE_TURRET_SAFE_X_CLOSE_BOUNDARY - TURRET_MOVING_CLOSE_SAFE_BUFFER,
                RED_TURRET_SAFE_X_CLOSE_BOUNDARY + TURRET_MOVING_CLOSE_SAFE_BUFFER
        );
    }

    public static double getMovingTurretSafeXFarBoundary() {
        return getAllianceFlipped(
                BLUE_TURRET_SAFE_X_FAR_BOUNDARY + TURRET_MOVING_FAR_SAFE_BUFFER,
                RED_TURRET_SAFE_X_FAR_BOUNDARY - TURRET_MOVING_FAR_SAFE_BUFFER
        );
    }
}