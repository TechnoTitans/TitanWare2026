package frc.robot.subsystems.intake.slide;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.SubsystemExt;
import frc.robot.utils.control.DeltaTime;
import org.littletonrobotics.junction.Logger;

public class IntakeSlide extends SubsystemExt {
    protected static final String LogKey = "IntakeSlide";
    private static final double PositionToleranceRots = 0.1;
    private static final double VelocityToleranceRotsPerSec = 0.02;

    private final HardwareConstants.IntakeSlideConstants constants;

    private final IntakeSlideIO intakeSlideIO;
    private final IntakeSlideIOInputsAutoLogged inputs = new IntakeSlideIOInputsAutoLogged();

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;


    private final DeltaTime deltaTime = new DeltaTime();
    private final TrapezoidProfile profile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(0.4, 0.3));
    private final TrapezoidProfile.State profileGoal = new TrapezoidProfile.State(0,0);
    private TrapezoidProfile.State profileSetpoint = new TrapezoidProfile.State(0,0);

    private ControlMode controlMode = ControlMode.HARD;

    public final Trigger atSlideSetpoint = new Trigger(this::atSetpoint);

    public enum Goal {
        STOW(0),
        EXTEND(3.4),
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

        atSlideSetpoint
                .onTrue(Commands.runOnce(() -> {
                    if (currentGoal == Goal.EXTEND) {
                        controlMode = ControlMode.SOFT;
                        intakeSlideIO.holdSlidePosition(currentGoal.positionSetpointRots);
                    }
                }))
                .onFalse(Commands.runOnce(() -> {
                    if (desiredGoal != currentGoal) {
                        controlMode = ControlMode.HARD;
                    }
                }));
    }

    @Override
    public void periodic() {
        final double intakeSlidePeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeSlideIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        final double dt = deltaTime.get();

        if (desiredGoal != currentGoal) {
            if (desiredGoal == Goal.SHOOTING) {
                profileSetpoint.position = inputs.averagePositionRots;
                profileSetpoint.velocity = inputs.averageVelocityRotsPerSec;

                profileGoal.position = desiredGoal.positionSetpointRots;
                profileGoal.velocity = 0;
            } else {
                intakeSlideIO.toSlidePosition(desiredGoal.positionSetpointRots);
            }

            currentGoal = desiredGoal;
        }

        if (currentGoal == Goal.SHOOTING) {
            profileSetpoint = profile.calculate(dt, profileSetpoint, profileGoal);
            intakeSlideIO.toSlidePositionUnprofiled(
                    profileSetpoint.position,
                    profileSetpoint.velocity
            );
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
                () -> setDesiredGoal(Goal.EXTEND)
        ).withName("ToGoal: " + goal.toString());
    }

    public Command setGoalCommand(final Goal goal) {
        return runOnce(() -> setDesiredGoal(goal)).withName("ToGoal: " + goal.toString());
    }

    public Rotation2d getIntakeSlidePositionRots() {
        return Rotation2d.fromRotations(inputs.averagePositionRots);
    }

    public boolean atGoal(final Goal goal) {
        return currentGoal == desiredGoal
                && atSetpoint();
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
        return inputs.masterPositionRots <= constants.reverseLimitRots();
    }

    private boolean atUpperLimit() {
        return inputs.masterPositionRots >= constants.forwardLimitRots();
    }

    public Command testIntakeSim() {
        return runOnce(intakeSlideIO::testIntakeSim);
    }
}
