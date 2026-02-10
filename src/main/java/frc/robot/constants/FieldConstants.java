package frc.robot.constants;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;

//Credit to FRC Team 6328
public class FieldConstants {
    //TODO: Redo field constants to match 2025 style
    public static final FieldType fieldType = FieldType.WELDED;

    // AprilTag related constants
    public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

    // Field dimensions
    public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
    public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

    public static final Translation2d ferryTarget  = Translation2d.kZero;


    /** Hub related constants */
    public static class Hub {

        // Dimensions
        public static final double width = Units.inchesToMeters(47.0);
        public static final double height =
                Units.inchesToMeters(72.0); // includes the catcher at the top
        public static final double innerWidth = Units.inchesToMeters(41.7);
        public static final double innerHeight = Units.inchesToMeters(56.5);

        // Relevant reference points on alliance side
        public static final Translation3d topCenterPoint =
                new Translation3d(
                        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
                        fieldWidth / 2.0,
                        height);

        public static final Translation2d hubCenterPoint = topCenterPoint.toTranslation2d();
    }

    /** Tower related constants */
    public static class Tower {
        // Dimensions
        public static final double width = Units.inchesToMeters(49.25);
        public static final double depth = Units.inchesToMeters(45.0);
        public static final double height = Units.inchesToMeters(78.25);
        public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
        public static final double frontFaceX = Units.inchesToMeters(43.51);

        public static final double uprightHeight = Units.inchesToMeters(72.1);

        // Rung heights from the floor
        public static final double lowRungHeight = Units.inchesToMeters(27.0);
        public static final double midRungHeight = Units.inchesToMeters(45.0);
        public static final double highRungHeight = Units.inchesToMeters(63.0);

        // Relevant reference points on alliance side
        public static final Translation2d centerPoint =
                new Translation2d(
                        frontFaceX, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY());
        public static final Translation2d leftUpright =
                new Translation2d(
                        frontFaceX,
                        (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                                + innerOpeningWidth / 2
                                + Units.inchesToMeters(0.75));
        public static final Translation2d rightUpright =
                new Translation2d(
                        frontFaceX,
                        (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                                - innerOpeningWidth / 2
                                - Units.inchesToMeters(0.75));

        // Relevant reference points on opposing side
        public static final Translation2d oppCenterPoint =
                new Translation2d(
                        fieldLength - frontFaceX,
                        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY());
        public static final Translation2d oppLeftUpright =
                new Translation2d(
                        fieldLength - frontFaceX,
                        (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                                + innerOpeningWidth / 2
                                + Units.inchesToMeters(0.75));
        public static final Translation2d oppRightUpright =
                new Translation2d(
                        fieldLength - frontFaceX,
                        (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                                - innerOpeningWidth / 2
                                - Units.inchesToMeters(0.75));
    }

    public static class Depot {
        // Dimensions
        public static final double width = Units.inchesToMeters(42.0);
        public static final double depth = Units.inchesToMeters(27.0);
        public static final double height = Units.inchesToMeters(1.125);
        public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

        // Relevant reference points on alliance side
        public static final Translation3d depotCenter =
                new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY, height);
        public static final Translation3d leftCorner =
                new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
        public static final Translation3d rightCorner =
                new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
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