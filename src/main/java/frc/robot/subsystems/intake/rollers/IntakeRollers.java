package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.SubsystemExt;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Objects;

public class IntakeRollers extends SubsystemExt {
    protected static final String LogKey = "IntakeRollers";

    public enum Goal {
        OFF(0),
        INTAKE(8);

        public final double volts;

        Goal(final double volts) {
            this.volts = volts;
        }
    }

    private enum InternalGoal {
        NONE,
        OFF(Goal.OFF),
        INTAKE(Goal.INTAKE);

        public static final HashMap<Goal, InternalGoal> GoalToInternal = new HashMap<>();
        static {
            for (final InternalGoal goal : InternalGoal.values()) {
                if (goal.goal != null) {
                    GoalToInternal.put(goal.goal, goal);
                }
            }
        }

        public static InternalGoal fromGoal(final Goal goal) {
            return Objects.requireNonNull(GoalToInternal.get(goal));
        }

        public final Goal goal;

        InternalGoal(final Goal goal) {
            this.goal = goal;
        }

        InternalGoal() {
            this(null);
        }
    }

    private final IntakeRollersIO intakeRollersIO;
    private final IntakeRollersIOInputsAutoLogged inputs;

    private InternalGoal desiredGoal = InternalGoal.OFF;

    public IntakeRollers(final Constants.RobotMode mode, final HardwareConstants.IntakeRollersConstants constants) {
        this.intakeRollersIO = switch (mode) {
            case REAL -> new IntakeRollersIOReal(constants);
            case SIM -> new IntakeRollersIOSim(constants);
            case REPLAY, DISABLED -> new IntakeRollersIO() {};
        };

        this.inputs = new IntakeRollersIOInputsAutoLogged();

        this.intakeRollersIO.config();
    }

    @Override
    public void periodic() {
        final double intakePeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeRollersIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - intakePeriodicUpdateStart)
        );
    }

    private void setVelocityImpl(final double volts) {
        intakeRollersIO.toIntakeVoltage(volts);
    }

    private void setGoalImpl(final Goal goal) {
        desiredGoal = InternalGoal.fromGoal(goal);
        setVelocityImpl(goal.volts);
    }

    public Command toInstantGoal(final Goal goal) {
        return runOnce(() -> setGoalImpl(goal));
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setGoalImpl(goal),
                () -> setGoalImpl(Goal.OFF)
        );
    }
}
