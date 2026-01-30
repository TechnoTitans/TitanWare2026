package frc.robot.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import javax.xml.crypto.dsig.Transform;

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

        Transform3d FRONT_LEFT_CAMERA = new Transform3d(
                new Translation3d(Units.inchesToMeters(-5.8), Units.inchesToMeters(-11.2), Units.inchesToMeters(18)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-90))
        );

        Transform3d FRONT_RIGHT_CAMERA = new Transform3d(
                new Translation3d(Units.inchesToMeters(-5.8), Units.inchesToMeters(11.2), Units.inchesToMeters(18)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(90))
        );

        Transform3d FRONT_CAMERA = new Transform3d(
                new Translation3d(Units.inchesToMeters(-6.8), Units.inchesToMeters(11.2), Units.inchesToMeters(18)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(180))
        );

        Transform3d BACK_CAMERA = new Transform3d(
                new Translation3d(Units.inchesToMeters(-5.4), Units.inchesToMeters(-10.5), Units.inchesToMeters(21)),
                new Rotation3d(0, Units.degreesToRadians(-22), Units.degreesToRadians(22))
        );
    }
}
