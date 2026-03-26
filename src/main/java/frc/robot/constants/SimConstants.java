package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
    }

    interface IntakeSlide {
        double MOMENT_OF_INERTIA = 0.0058;
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
    }
}
