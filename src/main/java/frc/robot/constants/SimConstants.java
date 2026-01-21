package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.constants.SwerveConstants;

public interface SimConstants {
    // Assume 2mOhm resistance for voltage drop calculation
    double FALCON_MOTOR_RESISTANCE = 0.002;

    interface CTRE {
        boolean DISABLE_NEUTRAL_MODE_IN_SIM = false;
        double CONFIG_TIMEOUT_SECONDS = 0.2;
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

    //TODO: Change ALL Number

    interface Hood {
        Translation3d TURRET_TO_HOOD_TRANSLATION = new Translation3d(Units.inchesToMeters(6.155), 0, Units.inchesToMeters(2.125));

        double LENGTH_METERS = Units.inchesToMeters(17);

        Rotation2d ZEROED_POSITION_TO_HORIZONTAL = Rotation2d.fromDegrees(15);
        Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0).plus(ZEROED_POSITION_TO_HORIZONTAL);
    }

    interface Shooter {
        double MOMENT_OF_INERTIA = 0.2;
    }

    interface Turret {
        Translation3d ORIGIN = new Translation3d(0, 0, Units.inchesToMeters(17.2));

        Transform2d TURRET_TO_ROBOT_TRANSFORM = new Transform2d(new Translation2d(0, 0), Rotation2d.kZero);

        double MOMENT_OF_INERTIA = 0.1068;
    }

    interface IntakeSlider {
        Pose3d RETRACTED_POSE = new Pose3d(new Translation3d(0, 0.142, 0.443), Rotation3d.kZero);

        Pose3d EXTENDED_POSE = new Pose3d(new Translation3d(0, -0.120, 0.345), Rotation3d.kZero);
    }

    interface Hopper {
        Pose3d RETRACTED_POSE = new Pose3d(new Translation3d(0, 0.087, 0.501),Rotation3d.kZero);

        Pose3d EXTENDED_POSE = new Pose3d(new Translation3d(0, -0.212, 0.501), Rotation3d.kZero);
    }
}
