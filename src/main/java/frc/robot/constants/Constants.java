package frc.robot.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public interface Constants {
    RobotMode CURRENT_MODE = RobotMode.SIM;
    CompetitionType CURRENT_COMPETITION_TYPE = CompetitionType.COMPETITION;
    double LOOP_PERIOD_SECONDS = 0.02;

    enum RobotMode {
        REAL,
        SIM,
        REPLAY,
        DISABLED
    }

    enum CompetitionType {
        TESTING,
        COMPETITION
    }

    interface NetworkTables {
        String AUTO_TABLE = "AutoSelector";
        String AUTO_PUBLISHER = "AutoOptions";
        String AUTO_SELECTED_SUBSCRIBER = "SelectedAuto";
    }

    interface Vision {
        Vector<N3> VISION_STD_DEV_COEFFS = VecBuilder.fill(0.02, 0.02, 0.02);
        double VISION_CAMERA_DEFAULT_STD_DEV_FACTOR = 1.0;

        double MAX_ACCEPT_BEST_POSE_AMBIGUITY = 0.15;

        Transform3d FRONT_HOPPER_CAMERA = new Transform3d(
                new Translation3d(Units.inchesToMeters(-4), Units.inchesToMeters(-11.5), Units.inchesToMeters(21.5)),
                new Rotation3d(0, Units.degreesToRadians(-22), Units.degreesToRadians(22))
        );

        Transform3d BACK_CENTER_CAMERA = new Transform3d(
                new Translation3d(Units.inchesToMeters(-6.8), Units.inchesToMeters(11.2), Units.inchesToMeters(18)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(180))
        );

        Transform3d BACK_LEFT_CAMERA = new Transform3d(
                new Translation3d(Units.inchesToMeters(-6.3), Units.inchesToMeters(11.2), Units.inchesToMeters(18)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(90))
        );

        Transform3d BACK_RIGHT_CAMERA = new Transform3d(
                new Translation3d(Units.inchesToMeters(-6.3), Units.inchesToMeters(-11.2), Units.inchesToMeters(18)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-90))
        );
    }
}
