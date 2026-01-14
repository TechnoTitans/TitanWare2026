package frc.robot.subsystems.superstructure.hopper;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    protected static final String LogKey = "Hopper";
    private final double PushingDesiredVelocity = 10;

    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged inputs;

    private double rollerVelocitySetpoint = 0.0;

    private boolean rollerPushing = false;

    public final Trigger isRollerPushing;

    public Hopper(final Constants.RobotMode mode, final HardwareConstants.HopperConstants constants) {
        this.hopperIO = switch (mode) {
            case REAL -> new HopperIOReal(constants);
            case SIM, REPLAY, DISABLED -> new HopperIO() {};
        };

        this.isRollerPushing = new Trigger(() -> rollerPushing);

        this.inputs = new HopperIOInputsAutoLogged();

        hopperIO.config();
    }

    @Override
    public void periodic() {
        final double hopperPeriodicFPGATime = RobotController.getFPGATime();

        hopperIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/RollerVelocitySetpoint", rollerVelocitySetpoint);

        Logger.recordOutput(LogKey + "/Triggers/IsRollerCollecting", isRollerPushing);

        Logger.recordOutput(LogKey + "/PeriodicIOPeriodMs", RobotController.getFPGATime() - hopperPeriodicFPGATime);
    }

    public Command collect() {
        return Commands.sequence(
                runOnce(() -> this.rollerPushing = true),
                toRollerVelocity(PushingDesiredVelocity)
        )
                .finallyDo(() -> this.rollerPushing = false)
                .withName("Collect");
    }

    private Command toRollerVelocity(final double rollerVelocity) {
        return runEnd(
                () -> {
                    this.rollerVelocitySetpoint = rollerVelocity;
                    hopperIO.toRollerVelocity(rollerVelocitySetpoint);
                },
                () -> {
                    this.rollerVelocitySetpoint = 0;
                    hopperIO.toRollerVelocity(rollerVelocitySetpoint);
                }
        ).withName("ToRollerVelocity");
    }
}
