package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public interface SimConstants {
    // Assume 2mOhm resistance for voltage drop calculation
    double MotorResistance = 0.002;

    interface CTRE {
        boolean DisableNeutralModeInSim = false;
        double ConfigTimeoutSeconds = 0.2;
    }

    interface Turret {
        Translation3d OriginOffset = new Translation3d(-0.127, 0, 0.386);
    }

    interface Hood {
        Translation3d TurretOffset = new Translation3d(0.121, 0, 0.054);
        Transform3d FuelExitOffset = new Transform3d(
                Units.inchesToMeters(-4.604),
                0,
                Units.inchesToMeters(-2.125),
                new Rotation3d(0, (-Math.PI / 2) + Units.degreesToRadians(10), 0)
        );
    }

    interface Shooter {
        double WheelRadiusMeters = Units.inchesToMeters(2);
        double WheelCircumferenceMeters = 2 * Math.PI * WheelRadiusMeters;
    }

    interface IntakeSlide {
        double DrivingGearDiameter = Units.inchesToMeters(1);
        double SlideRotationsToLinearDistanceMetersRatio = 2 * Math.PI * (DrivingGearDiameter / 2);

        Pose3d ExtendedPose = Pose3d.kZero;
        Pose3d RetractedPose = new Pose3d(
                Units.inchesToMeters(-10.616),
                0,
                Units.inchesToMeters(3.655),
                Rotation3d.kZero
        );
    }

    interface HopperExtension {
        Pose3d ExtendedPose = new Pose3d(
                Units.inchesToMeters(12.606),
                0,
                0,
                Rotation3d.kZero
        );
        Pose3d RetractedPose = Pose3d.kZero;
    }
}
