package frc.robot.constants;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;

//Credit to FRC Team 6328
public class FieldConstants {
    // Field dimensions
    public static final double FIELD_LENGTH_X_METERS = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
    public static final double FIELD_WIDTH_Y_METERS = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();
    public static final Pose2d RED_ORIGIN = new Pose2d(FIELD_LENGTH_X_METERS, FIELD_WIDTH_Y_METERS, Rotation2d.k180deg);

    /** Hub related constants */
    public static class Hub {
        public static final double WIDTH = Units.inchesToMeters(47.0);

        public static final Pose2d HUB_CENTER_BLUE =
                new Pose2d(new Translation2d(
                        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + WIDTH / 2.0,
                        FIELD_WIDTH_Y_METERS / 2.0), Rotation2d.kZero);

        public static final Pose2d HUB_CENTER_RED = HUB_CENTER_BLUE.relativeTo(RED_ORIGIN);
    }

    public static class Ferrying {
        public static final Pose2d TOP_FERRYING_BLUE =
                new Pose2d(new Translation2d(2,FIELD_WIDTH_Y_METERS - 2.0), Rotation2d.kZero);

        public static final Pose2d TOP_FERRYING_RED = TOP_FERRYING_BLUE.relativeTo(RED_ORIGIN);

        public static final Pose2d BOTTOM_FERRYING_BLUE =
                new Pose2d(new Translation2d(2, 2), Rotation2d.kZero);

        public static final Pose2d BOTTOM_FERRYING_RED = BOTTOM_FERRYING_BLUE.relativeTo(RED_ORIGIN);
    }

    public static class Climb {
        public static final Pose2d CLIMB_ALIGN_BLUE = new Pose2d(new Translation2d(2, 2), Rotation2d.k180deg);

        public static final Pose2d CLIMB_ALIGN_RED = CLIMB_ALIGN_BLUE.relativeTo(RED_ORIGIN);
    }

    public static class Depot {
        // Dimensions
        public static final double width = Units.inchesToMeters(42.0);
        public static final double depth = Units.inchesToMeters(27.0);
        public static final double height = Units.inchesToMeters(1.125);
        public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

        // Relevant reference points on alliance side
        public static final Translation3d depotCenter =
                new Translation3d(depth, (FIELD_WIDTH_Y_METERS / 2) + distanceFromCenterY, height);
        public static final Translation3d leftCorner =
                new Translation3d(depth, (FIELD_WIDTH_Y_METERS / 2) + distanceFromCenterY + (width / 2), height);
        public static final Translation3d rightCorner =
                new Translation3d(depth, (FIELD_WIDTH_Y_METERS / 2) + distanceFromCenterY - (width / 2), height);
    }

    public static class Outpost {
        // Dimensions
        public static final double width = Units.inchesToMeters(31.8);
        public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
        public static final double height = Units.inchesToMeters(7.0);

        // Relevant reference points on alliance side
        public static final Translation2d centerPoint =
                new Translation2d(0, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(29).get().getY());
    }

    private static <T> T getAllianceFlipped(final T blueAlliance, final T redAlliance) {
        return DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Red ? redAlliance : blueAlliance)
                .orElse(blueAlliance);
    }

    public static Translation2d getHubTarget() {
        return getAllianceFlipped(Hub.HUB_CENTER_BLUE, Hub.HUB_CENTER_RED).getTranslation();
    }

    public static Translation2d getFerryingTarget(final double robotYPositionMeters) {
        return (robotYPositionMeters > FIELD_WIDTH_Y_METERS/2 ?
                getAllianceFlipped(Ferrying.TOP_FERRYING_BLUE, Ferrying.TOP_FERRYING_RED)
                    : getAllianceFlipped(Ferrying.BOTTOM_FERRYING_BLUE, Ferrying.BOTTOM_FERRYING_RED)).getTranslation();
    }

    public static Pose2d getClimbTarget() {
        return getAllianceFlipped(Climb.CLIMB_ALIGN_BLUE, Climb.CLIMB_ALIGN_RED);
    }

    public enum FieldType {
        ANDYMARK("andymark"),
        WELDED("welded");

        private final String jsonFolder;

        FieldType(final String jsonFolder){
            this.jsonFolder = jsonFolder;
        }
    }

    public enum AprilTagLayoutType {
        OFFICIAL("2026-official"),
        NONE("2026-none");

        private final String name;
        private volatile AprilTagFieldLayout layout;
        private volatile String layoutString;

        AprilTagLayoutType(String name) {
            this.name = name;
        }

        public AprilTagFieldLayout getLayout() {
            if (layout == null) {
                synchronized (this) {
                    if (layout == null) {
                        try {
                            String p = Filesystem.getDeployDirectory().getPath() + "/2026-rebuilt-welded.json";
                            layout = new AprilTagFieldLayout(p);
                            layoutString = new ObjectMapper().writeValueAsString(layout);
                        } catch (IOException e) {
                            throw new RuntimeException(e);
                        }
                    }
                }
            }
            return layout;
        }

        public String getLayoutString() {
            if (layoutString == null) {
                getLayout();
            }
            return layoutString;
        }
    }

}