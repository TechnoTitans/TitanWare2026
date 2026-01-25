package frc.robot.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

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
    }
}
