package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    protected static final String LogKey = "Intake";
    private final double IntakingDesiredVelocity = 10;
    private final double EjectingDesiredVoltage = -5;

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs;

    private double rollerVelocitySetpoint = 0.0;
    private double rollerVoltageSetpoint = 0.0;

    private boolean rollerIntaking = false;
    private boolean rollerOuttaking = false;

    public final Trigger isRollerIntaking;
    public final Trigger isRollerOuttaking;

    public Intake(final Constants.RobotMode mode, final HardwareConstants.IntakeConstants constants) {
        this.intakeIO = switch (mode) {
            case REAL -> new IntakeIOReal(constants);
            case SIM,
                 REPLAY, DISABLED -> new IntakeIO() {};
        };

        this.isRollerIntaking = new Trigger(() -> rollerIntaking);
        this.isRollerOuttaking = new Trigger(() -> rollerOuttaking);

        this.inputs = new IntakeIOInputsAutoLogged();

        intakeIO.config();
    }

    @Override
    public void periodic() {
        final double IntakePeriodicUpdateStart = Timer.getFPGATimestamp();;

        intakeIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/RollerVelocitySetpoint", rollerVelocitySetpoint);
        Logger.recordOutput(LogKey + "/RollerVoltageSetpoint", rollerVoltageSetpoint);

        Logger.recordOutput(LogKey + "/Triggers/IsRollerIntaking", isRollerIntaking);

        Logger.recordOutput(LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - IntakePeriodicUpdateStart));
    }

    public Command intake() {
        return Commands.sequence(
                        runOnce(() -> this.rollerIntaking = true),
                        toRollerVelocity(IntakingDesiredVelocity)
                )
                .finallyDo(() -> this.rollerIntaking = false)
                .withName("Intake");
    }

    public Command eject() {
        return Commands.sequence(
                        runOnce(() -> this.rollerOuttaking = true),
                        toRollerVoltage(EjectingDesiredVoltage)
                )
                .finallyDo(() -> this.rollerOuttaking = false)
                .withName("Eject");
    }

    private Command toRollerVelocity(final double rollerVelocity) {
        return runEnd(
                () -> {
                    this.rollerVelocitySetpoint = rollerVelocity;
                    intakeIO.toRollerVelocity(rollerVelocitySetpoint);
                },
                () -> {
                    this.rollerVelocitySetpoint = 0;
                    intakeIO.toRollerVelocity(rollerVelocitySetpoint);
                }
        ).withName("ToRollerVelocity");
    }

    private Command toRollerVoltage(final double volts) {
        return runEnd(
                () -> {
                    this.rollerVoltageSetpoint = volts;
                    intakeIO.toRollerVoltage(volts);
                },
                () -> {
                    this.rollerVoltageSetpoint = 0;
                    intakeIO.toRollerVoltage(0);
                }
        ).withName("ToRollerVoltage");
    }
}