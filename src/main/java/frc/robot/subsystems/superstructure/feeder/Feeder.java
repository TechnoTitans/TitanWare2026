package frc.robot.subsystems.superstructure.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
    protected static final String LogKey = "Superstructure/Feeder";
    private static final double VelocityToleranceRotsPerSec = 0.002;

    private final FeederIO feederIO;
    private final FeederIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOP;
    private Goal currentGoal = desiredGoal;

    public enum Goal {
        STOP(0),
        FEED(5);

        private final double rollerVelocitySetpoint;

        Goal(final double rollerVelocitySetpoint) {
            this.rollerVelocitySetpoint = rollerVelocitySetpoint;
        }

        public double getRollerVelocitySetpoint() {
            return rollerVelocitySetpoint;
        }
    }

    public Feeder(final Constants.RobotMode mode, final HardwareConstants.FeederConstants constants) {
        this.feederIO = switch (mode) {
            case REAL -> new FeederIOReal(constants);
            case SIM -> new FeederIOSim(constants);
            case DISABLED, REPLAY -> new FeederIO() {};
        };

        this.inputs = new FeederIOInputsAutoLogged();

        feederIO.config();

        feederIO.toRollerVelocity(desiredGoal.getRollerVelocitySetpoint());
    }

    @Override
    public void periodic() {
        final double FeederPeriodicFPGATime = Timer.getFPGATimestamp();

        feederIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            feederIO.toRollerVelocity(desiredGoal.getRollerVelocitySetpoint());
            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/FeederVelocityRotsPerSec", desiredGoal.getRollerVelocitySetpoint());
        Logger.recordOutput(LogKey + "/Triggers/AtVelocitySetpoint", atVelocitySetpoint());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - FeederPeriodicFPGATime)
        );
    }

    public void setGoal(final Goal desiredGoal) {
        this.desiredGoal = desiredGoal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    private boolean atVelocitySetpoint() {
        return MathUtil.isNear(desiredGoal.rollerVelocitySetpoint, inputs.rollerVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }
}
