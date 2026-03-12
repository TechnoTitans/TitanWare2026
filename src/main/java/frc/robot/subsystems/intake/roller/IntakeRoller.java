package frc.robot.subsystems.intake.roller;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {
    protected static final String LogKey = "IntakeRoller";

    private final IntakeRollerIO intakeRollerIO;
    private final IntakeRollerIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOP;
    private Goal currentGoal = desiredGoal;

    public enum Goal {
        STOP(0),
        INTAKE(8);

        private final double voltageSetpoint;

        Goal(final double voltageSetpoint) {
            this.voltageSetpoint = voltageSetpoint;
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
        final double intakeRollerPeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeRollerIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            intakeRollerIO.toRollerVoltage(desiredGoal.voltageSetpoint);

            currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/VoltageSetpoint", desiredGoal.voltageSetpoint);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - intakeRollerPeriodicUpdateStart)
        );
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> setGoal(goal),
                () -> setGoal(Goal.STOP)
        ).withName("ToGoal: " + goal.toString());
    }

    public Command setGoal(final Goal goal) {
        return runOnce(() -> setDesiredGoal(goal))
                .withName("SetGoal: " + goal.toString());
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }
}
