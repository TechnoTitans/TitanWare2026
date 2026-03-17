package frc.robot.constants;

public interface SimConstants {
    // TODO: Make constant for sim periodic so that every subsystem uses same


    // Assume 2mOhm resistance for voltage drop calculation
    double MotorResistance = 0.002;

    interface CTRE {
        double ConfigTimeoutSeconds = 0.2;
    }
    interface Shooter {
        double MOMENT_OF_INERTIA = 0.01;
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
    }
}
