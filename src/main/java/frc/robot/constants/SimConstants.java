package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.constants.SwerveConstants;

public interface SimConstants {
    // TODO: Make constant for sim periodic so that every subsystem uses same


    // Assume 2mOhm resistance for voltage drop calculation
    double MotorResistance = 0.002;

    interface CTRE {
        double ConfigTimeoutSeconds = 0.2;
    }
    interface Shooter {
        double MOMENT_OF_INERTIA = 0.2;
    }

    interface Turret {
        double MOMENT_OF_INERTIA = 0.1068;
    }

    interface IntakeSlide {
        double MOMENT_OF_INERTIA = 0.0058;
    }
}
