package frc.robot.subsystems.intake.slide;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSlide extends SubsystemBase {
    protected static final String LogKey = "IntakeSlide";
    private static final double PositionToleranceRots = 0.1;
    private static final double VelocityToleranceRotsPerSec = 0.02;

    private final HardwareConstants.IntakeSlideConstants constants;

    private final IntakeSlideIO intakeSlideIO;
    private final IntakeSlideIOInputsAutoLogged inputs = new IntakeSlideIOInputsAutoLogged();

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;

    //TODO: Need to implement
    private ControlMode controlMode = ControlMode.HARD;

    public final Trigger atSlideSetpoint = new Trigger(this::atSetpoint)
            .onTrue(Commands.runOnce(() -> {
                if (currentGoal == Goal.INTAKE) {
                    controlMode = ControlMode.SOFT;
                }
            }))
            .onFalse(Commands.runOnce(() -> {
                if (desiredGoal != currentGoal) {
                    controlMode = ControlMode.HARD;
                }
            }));

    public enum Goal {
        STOW(0),
        INTAKE(3.8),
        SHOOTING(0);

        private final double positionSetpointRots;

        Goal(final double positionSetpointRots) {
            this.positionSetpointRots = positionSetpointRots;
        }
    }

    public enum ControlMode {
        SOFT,
        HARD
    }

    public IntakeSlide(final Constants.RobotMode mode, final HardwareConstants.IntakeSlideConstants constants) {
        this.constants = constants;

        this.intakeSlideIO = switch (mode) {
            case REAL -> new IntakeSlideIOReal(constants);
            case SIM -> new IntakeSlideIOSim(constants);
            case REPLAY, DISABLED -> new IntakeSlideIO() {};
        };

        intakeSlideIO.zeroMotors();
    }

    @Override
    public void periodic() {
        final double intakeSlidePeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeSlideIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            switch (controlMode) {
                case SOFT -> intakeSlideIO.holdSlidePosition(desiredGoal.positionSetpointRots);
                case HARD -> {
                    if (desiredGoal == Goal.SHOOTING) {
                        intakeSlideIO.toSlidePositionUnprofiled(
                                desiredGoal.positionSetpointRots,
                                -0.1
                        );
                    } else {
                        intakeSlideIO.toSlidePosition(desiredGoal.positionSetpointRots);
                    }
                }
            }
            currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/PositionSetpointRots", desiredGoal.positionSetpointRots);

        Logger.recordOutput(LogKey + "/ControlMode", controlMode);

        Logger.recordOutput(LogKey + "/Triggers/AtPositionSetpoint", atSetpoint());
        Logger.recordOutput(LogKey + "/Triggers/AtSlideLowerLimit", atLowerLimit());
        Logger.recordOutput(LogKey + "/Triggers/AtSlideUpperLimit", atUpperLimit());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - intakeSlidePeriodicUpdateStart)
        );
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.INTAKE)
        ).withName("ToGoal: " + goal.toString());
    }

    public Command setGoal(final Goal goal) {
        return runOnce(
                () -> setDesiredGoal(goal)
        ).withName("ToGoal: " + goal.toString());
    }

    public Rotation2d getIntakeSlidePositionRots() {
        return Rotation2d.fromRotations(inputs.averagePositionRots);
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    private boolean atSetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(desiredGoal.positionSetpointRots, inputs.masterPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.masterVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }

    private boolean atLowerLimit() {
        return inputs.masterPositionRots <= constants.lowerLimitRots();
    }

    private boolean atUpperLimit() {
        return inputs.masterPositionRots >= constants.upperLimitRots();
    }
}
