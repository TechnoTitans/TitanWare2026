package frc.robot.subsystems.indexer.feeder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.ext.SubsystemExt;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemExt {
    protected static final String LogKey = "Feeder";

    public enum Goal {
        BACK_OUT(-60),
        OFF(0),
        FEED(60);

        private final double torqueCurrentAmps;

        Goal(final double torqueCurrentAmps) {
            this.torqueCurrentAmps = torqueCurrentAmps;
        }
    }

    private final FeederIO feederIO;
    private final FeederIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.OFF;
    private double torqueCurrentSetpointAmps = 0.0;

    public Feeder(final Constants.RobotMode mode, final HardwareConstants.FeederConstants constants) {
        this.feederIO = switch (mode) {
            case REAL -> new FeederIOReal(constants);
            case SIM -> new FeederIOSim(constants);
            case DISABLED, REPLAY -> new FeederIO() {
            };
        };

        this.inputs = new FeederIOInputsAutoLogged();

        this.feederIO.config();
        this.feederIO.toWheelTorqueCurrent(desiredGoal.torqueCurrentAmps);
    }

    @Override
    public void periodic() {
        final double feederPeriodicFPGATime = Timer.getFPGATimestamp();

        feederIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/TorqueCurrentSetpointAmps", torqueCurrentSetpointAmps);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - feederPeriodicFPGATime)
        );
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.OFF)
        ).withName("ToGoal: " + goal);
    }


    private void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        setDesiredTorqueCurrent(goal.torqueCurrentAmps);
    }

    private void setDesiredTorqueCurrent(final double torqueCurrentAmps) {
        torqueCurrentSetpointAmps = torqueCurrentAmps;
        feederIO.toWheelTorqueCurrent(torqueCurrentAmps);
    }
}
