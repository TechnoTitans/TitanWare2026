package frc.robot.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.LoggedTrigger;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
    protected static final String LogKey = "Feeder";
    private static final double VelocityToleranceRotsPerSec = 0.05;

    private final FeederIO feederIO;
    private final FeederIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOP;
    private Goal currentGoal = desiredGoal;

    private final LinearFilter currentFilter;

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private boolean feeding = false;
    public final LoggedTrigger isFeeding = group.t("isFeeding", () -> feeding);

    public enum Goal {
        STOP(0),
        BACK_OUT(-60),
        FEED(60);

        private final double torqueCurrentSetpoint;

        Goal(final double torqueCurrentSetpoint) {
            this.torqueCurrentSetpoint = torqueCurrentSetpoint;
        }
    }

    public Feeder(final Constants.RobotMode mode, final HardwareConstants.FeederConstants constants) {
        this.feederIO = switch (mode) {
            case REAL -> new FeederIOReal(constants);
            case SIM -> new FeederIOSim(constants);
            case DISABLED, REPLAY -> new FeederIO() {};
        };

        this.inputs = new FeederIOInputsAutoLogged();

        this.currentFilter = LinearFilter.movingAverage(3);

        feederIO.config();
        feederIO.toWheelTorqueCurrent(desiredGoal.torqueCurrentSetpoint);
    }

    @Override
    public void periodic() {
        final double feederPeriodicFPGATime = Timer.getFPGATimestamp();

        feederIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            feeding = desiredGoal == Goal.FEED;
            feederIO.toWheelTorqueCurrent(desiredGoal.torqueCurrentSetpoint);
            currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/TorqueCurrentSetpointAmps", desiredGoal.torqueCurrentSetpoint);
        Logger.recordOutput(LogKey + "/AtSetpoint", atSetpoint());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - feederPeriodicFPGATime)
        );
    }

    public double getFilteredCurrent() {
        return currentFilter.calculate(inputs.wheelTorqueCurrentAmps);
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.STOP)
        ).withName("ToGoal: " + goal.toString());
    }

    public void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", goal.toString());
    }

    private boolean atSetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(desiredGoal.torqueCurrentSetpoint, inputs.wheelVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }
}
