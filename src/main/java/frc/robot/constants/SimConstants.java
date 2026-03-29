package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

public interface SimConstants {
    double SIM_UPDATE_PERIODIC_SEC = 0.005;

    // Assume 2mOhm resistance for voltage drop calculation
    double MotorResistance = 0.002;

    interface CTRE {
        double ConfigTimeoutSeconds = 0.2;
    }
    interface Shooter {
        double MOMENT_OF_INERTIA = 0.01;

        double WheelRadiusMeters = Units.inchesToMeters(2);
        double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WheelRadiusMeters;
    }

    interface Turret {
        double MOMENT_OF_INERTIA = 0.09973;
        Translation3d ORIGIN = new Translation3d(-0.127, 0, 0.386);
    }

    interface IntakeSlide {
        double MOMENT_OF_INERTIA = 0.0058;
        Pose3d RETRACTED_POSE = new Pose3d(new Translation3d(-0.142, 0, 0.443), Rotation3d.kZero);
        Pose3d EXTENDED_POSE = new Pose3d(new Translation3d(0.120, 0, 0.345), Rotation3d.kZero);
    }

    interface Hopper {
        Pose3d RETRACTED_POSE = new Pose3d(new Translation3d(-0.087, 0, 0.501), Rotation3d.kZero);
        Pose3d EXTENDED_POSE = new Pose3d(new Translation3d(0.212, 0, 0.501), Rotation3d.kZero);
    }

    interface Feeder {
        double MOMENT_OF_INERTIA = 0.0026;
    }

    interface Hood {
        double MOMENT_OF_INERTIA = 0.04;

        Transform3d FuelExitOffset = new Transform3d(
                Units.inchesToMeters(-4.604),
                0,
                Units.inchesToMeters(-2.125),
                new Rotation3d(0, (-Math.PI / 2) + Units.degreesToRadians(10), 0)
        );
        Translation3d TURRET_TO_HOOD_TRANSLATION = new Translation3d(0.121, 0, 0.054);
    }
}
