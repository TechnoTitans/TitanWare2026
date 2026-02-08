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
        Translation3d TURRET_TO_HOOD_TRANSLATION = new Translation3d(0,0.121, 0.0715);

        double LENGTH_METERS = Units.inchesToMeters(9.4);

        Rotation2d ZEROED_POSITION_TO_HORIZONTAL = Rotation2d.fromDegrees(25.9);
        Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0).plus(ZEROED_POSITION_TO_HORIZONTAL);
    }

    interface Shooter {
        double MOMENT_OF_INERTIA = 0.2;
    }

    interface Turret {
        Translation3d ORIGIN = new Translation3d(-0.127, 0, 0.384);

        Transform2d TURRET_TO_ROBOT_TRANSFORM = new Transform2d(ORIGIN.toTranslation2d(), Rotation2d.kZero);

        double MOMENT_OF_INERTIA = 0.1068;
    }

    interface IntakeSlide {
        Pose3d RETRACTED_POSE = new Pose3d(new Translation3d(-0.142, 0, 0.443), Rotation3d.kZero);

        Pose3d EXTENDED_POSE = new Pose3d(new Translation3d(0.120,0, 0.345), Rotation3d.kZero);
    }

    interface Hopper {
        Pose3d RETRACTED_POSE = new Pose3d(new Translation3d(-0.087, 0, 0.501),Rotation3d.kZero);

        Pose3d EXTENDED_POSE = new Pose3d(new Translation3d(0.212,0, 0.501), Rotation3d.kZero);
    }

    interface Climb {
        double MASS_KG = 10;

        Pose3d ORIGIN = new Pose3d(new Translation3d(-0.162, 0.193, 0.104), Rotation3d.kZero);

        Rotation3d ANGLE_FROM_HORIZONTAL = new Rotation3d(Units.degreesToRadians(-114.495), 0, 0);

        double STAGE_1_MAX_EXTENSION = Units.inchesToMeters(9.160);

        double STAGE_2_MAX_EXTENSION = Units.inchesToMeters(7.963);
    }
}
