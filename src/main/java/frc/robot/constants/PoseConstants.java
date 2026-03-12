package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

public interface PoseConstants {
    interface Hood {
        Translation3d TURRET_TO_HOOD_TRANSLATION = new Translation3d(0, 0.121, 0.0715);
        Rotation2d ZEROED_POSITION_TO_HORIZONTAL = Rotation2d.fromDegrees(25.9);
    }

    interface Turret {
        Translation3d ORIGIN = new Translation3d(-0.127, 0, 0.384);
        Transform2d ROBOT_TO_TURRET_TRANSFORM_2D = new Transform2d(ORIGIN.toTranslation2d(), Rotation2d.kZero);
        Rotation3d TURRET_ZERO_OFFSET = new Rotation3d(Rotation2d.k180deg);
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