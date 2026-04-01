package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.ext.SubsystemExt;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemExt {
    protected static final String LogKey = "IntakeRollers";

    public enum Goal {
        OFF(0),
        INTAKE(8);

        private final double volts;

        Goal(final double volts) {
            this.volts = volts;
        }
    }

    private final IntakeRollersIO intakeRollersIO;
    private final IntakeRollerIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.OFF;
    private double voltageSetpoint = 0.0;

    public IntakeRollers(final Constants.RobotMode mode, final HardwareConstants.IntakeRollerConstants constants) {
        this.intakeRollersIO = switch (mode) {
            case REAL -> new IntakeRollersIOReal(constants);
            case SIM -> new IntakeRollersIOSim(constants);
            case REPLAY, DISABLED -> new IntakeRollersIO() {};
        };

        this.inputs = new IntakeRollerIOInputsAutoLogged();
        this.intakeRollersIO.config();
    }

    @Override
    public void periodic() {
        final double intakeRollersPeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeRollersIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/VoltageSetpoint", voltageSetpoint);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - intakeRollersPeriodicUpdateStart)
        );
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.OFF)
        ).withName("ToGoal: " + goal);
    }

    public Command setGoal(final Goal goal) {
        return runOnce(() -> setDesiredGoal(goal))
                .withName("SetGoal: " + goal);
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        setDesiredVelocity(goal.volts);
    }

    private void setDesiredVelocity(final double volts) {
        voltageSetpoint = volts;
        intakeRollersIO.toRollersVoltage(volts);
    }
}
