package frc.robot.subsystems.intake.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {
    protected static final String LogKey = "/Intake/Roller";
    private static final double VelocityToleranceRotsPerSec = 0.02;

    private final IntakeRollerIO intakeRollerIO;
    private final IntakeRollerIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOP;
    private Goal currentGoal = desiredGoal;

    public enum Goal {
        STOP(0),
        INTAKE(30);

        private final double rollerVelocityGoalRotsPerSec;

        Goal(final double rollerVelocityGoalRotsPerSec) {
            this.rollerVelocityGoalRotsPerSec = rollerVelocityGoalRotsPerSec;
        }

        public double getRollerVelocityGoalRotsPerSec() {
            return rollerVelocityGoalRotsPerSec;
        }
    }

    public IntakeRoller(final Constants.RobotMode mode, final HardwareConstants.IntakeRollerConstants constants) {
        this.intakeRollerIO = switch (mode) {
            case REAL -> new IntakeRollerIOReal(constants);
            case SIM -> new IntakeRollerIOSim(constants);
            case REPLAY, DISABLED -> new IntakeRollerIO() {};
        };

        this.inputs = new IntakeRollerIOInputsAutoLogged();
        this.intakeRollerIO.config();
    }

    @Override
    public void periodic() {
        final double IntakeRollerPeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeRollerIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            intakeRollerIO.toRollerVelocity(desiredGoal.getRollerVelocityGoalRotsPerSec());

            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());

        Logger.recordOutput(LogKey + "/DesiredGoal/RollerVelocity", desiredGoal.getRollerVelocityGoalRotsPerSec());

        Logger.recordOutput(LogKey + "/Roller/AtRollerVelocitySetpoint", atRollerVelocitySetpoint());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - IntakeRollerPeriodicUpdateStart)
        );
    }

    public boolean atRollerVelocitySetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(desiredGoal.rollerVelocityGoalRotsPerSec, inputs.rollerVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }

    public boolean isIntaking() {
        return currentGoal == Goal.INTAKE;
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> setGoal(goal),
                () -> setGoal(Goal.STOP)
        ).withName("ToGoal: " + goal);
    }

    public Command setGoal(final Goal goal) {
        return runOnce(() -> setDesiredGoal(goal))
                .withName("SetGoal: " + goal);
    }

    private void setDesiredGoal(final Goal goal) {
        this.desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }
}
