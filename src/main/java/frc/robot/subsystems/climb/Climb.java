package frc.robot.subsystems.climb;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Climb extends SubsystemBase {
    protected static final String LogKey = "Climb";
    private static final double PositionToleranceRots = 0.03;
    private static final double VelocityToleranceRotsPerSec = 0.03;

    private final HardwareConstants.ClimbConstants constants;

    private final ClimbIO climbIO;
    private final ClimbIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSetpoint = new Trigger(this::atPositionSetpoint);
    public final Trigger atLowerLimit = new Trigger(this::atLowerLimit);
    public final Trigger atUpperLimit = new Trigger(this::atUpperLimit);

    public enum Goal {
        STOW(0),
        EXTEND(2);

        private final double positionGoalMeters;
        Goal(final double positionGoalMeters) {
            this.positionGoalMeters = positionGoalMeters;
        }

        public double getPositionGoalRots(final HardwareConstants.ClimbConstants constants) {
            return positionGoalMeters / constants.spoolDiameterMeters();
        }

        public double getPositionGoalMeters() {
            return positionGoalMeters;
        }
    }

    public Climb(
            final Constants.RobotMode mode,
            final HardwareConstants.ClimbConstants constants
    ) {
        this.constants = constants;

        this.climbIO = switch (mode) {
            case REAL -> new ClimbIOReal(constants);
            case SIM -> new ClimbIOSim(constants);
            case REPLAY, DISABLED -> new ClimbIO() {};
        };

        this.inputs = new ClimbIOInputsAutoLogged();

        this.climbIO.config();
        this.zero();
    }

    @Override
    public void periodic() {
        final double climbPeriodicUpdateStart = Timer.getFPGATimestamp();

        climbIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            climbIO.toPosition(desiredGoal.getPositionGoalRots(constants));
            this.currentGoal = desiredGoal;
        }

        //Todo:
        // Might need to add something like ElevatorArm last year where we control climb diff when climbing
        // Might also need to slow down when unclimbing

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/DesiredGoal/ClimbPositionRots", desiredGoal.getPositionGoalRots(constants));
        Logger.recordOutput(LogKey + "/ExtensionMeters", getExtensionMeters());
        Logger.recordOutput(LogKey + "/Triggers/AtLowerLimit", atLowerLimit());
        Logger.recordOutput(LogKey + "/Triggers/AtUpperLimit", atUpperLimit());
        Logger.recordOutput(LogKey + "/Triggers/AtSetpoint", atSetpoint);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - climbPeriodicUpdateStart)
        );
    }

    public boolean atGoal(final Goal goal) {
        return MathUtil.isNear(goal.getPositionGoalRots(constants), inputs.motorPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.motorVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }

    private boolean atPositionSetpoint() {
        double goalRots = desiredGoal.getPositionGoalRots(constants);
        return MathUtil.isNear(goalRots, inputs.motorPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.motorVelocityRotsPerSec, VelocityToleranceRotsPerSec)
                && currentGoal == desiredGoal;
    }

    public double getExtensionMeters() {
        return inputs.motorPositionRots;
    }

    public void zero() {
        this.climbIO.setPosition(0);
    }

    public boolean isExtended() {
        return atGoal(Goal.EXTEND);
    }

    public Command toGoal(final Goal goal){
        return runEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.STOW)
        ).withName("ToGoal: " + goal);
    }

    public Command setGoal(final Goal goal) {
        return runOnce(
                () -> setDesiredGoal(goal)
        ).withName("SetGoal: " + goal);
    }

    private void setDesiredGoal(final Goal goal) {
        this.desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    private boolean atLowerLimit() {
        return inputs.motorPositionRots <= constants.lowerLimitRots();
    }

    private boolean atUpperLimit() {
        return inputs.motorPositionRots >= constants.upperLimitRots();
    }
}
