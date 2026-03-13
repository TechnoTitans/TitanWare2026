package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

public interface PoseConstants {
    interface Hood {
        Translation3d TURRET_TO_HOOD_TRANSLATION = new Translation3d(0.121, 0, 0.054);
        Transform3d FuelExitOffset = new Transform3d(
                Units.inchesToMeters(-4.604),
                0,
                Units.inchesToMeters(-2.125),
                new Rotation3d(0, (-Math.PI / 2) + Units.degreesToRadians(10), 0)
        );
    }

    interface Turret {
        Translation3d ORIGIN = new Translation3d(-0.127, 0, 0.386);
        Transform2d ROBOT_TO_TURRET_TRANSFORM_2D = new Transform2d(ORIGIN.toTranslation2d(), Rotation2d.kZero);
    }

    interface IntakeSlide {
        Pose3d RETRACTED_POSE = new Pose3d(new Translation3d(-0.142, 0, 0.443), Rotation3d.kZero);
        Pose3d EXTENDED_POSE = new Pose3d(new Translation3d(0.120, 0, 0.345), Rotation3d.kZero);
    }

    interface Hopper {
        Pose3d RETRACTED_POSE = new Pose3d(new Translation3d(-0.087, 0, 0.501), Rotation3d.kZero);
        Pose3d EXTENDED_POSE = new Pose3d(new Translation3d(0.212, 0, 0.501), Rotation3d.kZero);
    }
}