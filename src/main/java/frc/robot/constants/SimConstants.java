package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.constants.SwerveConstants;

public interface SimConstants {
    double SIM_UPDATE_PERIOD_SEC = 0.005;

    // Assume 2mOhm resistance for voltage drop calculation
    double MotorResistance = 0.002;

    interface CTRE {
        boolean DisableNeutralModeInSim = false;
        double ConfigTimeoutSeconds = 0.2;
    }

    interface SwerveModules {
        double WHEEL_RADIUS_M = SwerveConstants.Config.wheelRadiusMeters();
        double WHEEL_MASS_KG = 0.2313321; //0.51 lbs
        double DRIVE_WHEEL_MOMENT_OF_INERTIA_KG_M_SQUARED = WHEEL_MASS_KG * WHEEL_RADIUS_M * WHEEL_RADIUS_M;
        double TURN_WHEEL_MOMENT_OF_INERTIA_KG_M_SQUARED = 0.004;

        /**
         * Simulated drive voltage required to overcome friction.
         */
        double DRIVE_KS_VOLTS = 0.25;
        /**
         * Simulated steer voltage required to overcome friction.
         */
        double STEER_KS_VOLTS = 0.25;
    }

    interface Hood {
        double LENGTH_METERS = Units.inchesToMeters(9.4);

        Translation3d TURRET_OFFSET = new Translation3d(0.121, 0, 0.054);
    }

    interface Shooter {
        double MOMENT_OF_INERTIA = 0.2;
    }

    interface Turret {
        double MOMENT_OF_INERTIA = 0.1068;

        Translation3d ORIGIN_OFFSET = new Translation3d(-0.127, 0, 0.386);
    }

    interface Hopper {
        Pose3d EXTENDED_POSE = new Pose3d(Units.inchesToMeters(12.606), 0, 0, Rotation3d.kZero);
        Pose3d RETRACTED_POSE = Pose3d.kZero;
    }

    interface IntakeSlide {
        double MOMENT_OF_INERTIA = 0.0058;

        double DrivingGearDiameter = Units.inchesToMeters(1);
        double SlideRotationsToLinearDistanceMetersRatio = 2 * Math.PI * (DrivingGearDiameter / 2);

        Pose3d EXTENDED_POSE = Pose3d.kZero;
        Pose3d RETRACTED_POSE = new Pose3d(
                Units.inchesToMeters(-10.616),
                0,
                Units.inchesToMeters(3.655),
                Rotation3d.kZero
        );
    }
}
