package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

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

    private static final double TRENCH_WIDTH = Units.inchesToMeters(6);
    private static final double TURRET_SAFE_BUFFER = Units.inchesToMeters(12);
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

    public static final Pose2d TOP_FERRYING_BLUE =
            new Pose2d(new Translation2d(2, FIELD_WIDTH_Y_METERS - 2.0), Rotation2d.kZero);

    public static final Pose2d TOP_FERRYING_RED = TOP_FERRYING_BLUE.relativeTo(RED_ORIGIN);

    public static final Pose2d BOTTOM_FERRYING_BLUE =
            new Pose2d(new Translation2d(2, 2), Rotation2d.kZero);

    public static final Pose2d BOTTOM_FERRYING_RED = BOTTOM_FERRYING_BLUE.relativeTo(RED_ORIGIN);

    private static <T> T getAllianceFlipped(final T blueAlliance, final T redAlliance) {
        return DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Red ? redAlliance : blueAlliance)
                .orElse(blueAlliance);
    }

    public static Translation2d getHubTarget() {
        return getAllianceFlipped(BLUE_HUB_POSE, RED_HUB_POSE).getTranslation();
    }

    public static Translation2d getFerryingTarget(final double robotYPositionMeters) {
        return (robotYPositionMeters > FIELD_WIDTH_Y_METERS / 2.0 ?
                getAllianceFlipped(TOP_FERRYING_BLUE, TOP_FERRYING_RED)
                : getAllianceFlipped(BOTTOM_FERRYING_BLUE, BOTTOM_FERRYING_RED)).getTranslation();
    }

    public static double getTurretSafeXCloseBoundary() {
        return getAllianceFlipped(BLUE_TURRET_SAFE_X_CLOSE_BOUNDARY, RED_TURRET_SAFE_X_CLOSE_BOUNDARY);
    }

    public static double getTurretSafeXFarBoundary() {
        return getAllianceFlipped(BLUE_TURRET_SAFE_X_FAR_BOUNDARY, RED_TURRET_SAFE_X_FAR_BOUNDARY);
    }
}