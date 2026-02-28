package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

public interface PoseConstants {
    interface Hood {
        Translation3d TURRET_TO_HOOD_TRANSLATION = new Translation3d(0, 0.121, 0.0715);
        Rotation2d ZEROED_POSITION_TO_HORIZONTAL = Rotation2d.fromDegrees(25.9);
        Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0).plus(ZEROED_POSITION_TO_HORIZONTAL);
    }

    interface Turret {
        Translation3d ORIGIN = new Translation3d(-0.127, 0, 0.384);
        Transform2d ROBOT_TO_TURRET_TRANSFORM_2D = new Transform2d(ORIGIN.toTranslation2d(), Rotation2d.kZero);
        Transform3d ROBOT_TO_TURRET_TRANSFORM_3D = new Transform3d(ORIGIN, new Rotation3d(Rotation2d.kCCW_90deg));
        Rotation3d TURRET_ZERO_OFFSET = new Rotation3d(Rotation2d.kCW_90deg);
    }

    interface IntakeSlide {
        Pose3d RETRACTED_POSE = new Pose3d(new Translation3d(-0.142, 0, 0.443), Rotation3d.kZero);
        Pose3d EXTENDED_POSE = new Pose3d(new Translation3d(0.120, 0, 0.345), Rotation3d.kZero);
    }

    interface Hopper {
        Pose3d RETRACTED_POSE = new Pose3d(new Translation3d(-0.087, 0, 0.501), Rotation3d.kZero);
        Pose3d EXTENDED_POSE = new Pose3d(new Translation3d(0.212, 0, 0.501), Rotation3d.kZero);
    }

    interface Climb {
        Translation3d ORIGIN = new Translation3d(-0.162, -0.176, 0.068);
        Rotation3d ANGLE_FROM_HORIZONTAL = new Rotation3d(0, Units.degreesToRadians(-24.496), 0);
        double STAGE_1_MAX_EXTENSION = Units.inchesToMeters(9.160);
        double STAGE_2_MAX_EXTENSION = Units.inchesToMeters(7.963);
    }
}
