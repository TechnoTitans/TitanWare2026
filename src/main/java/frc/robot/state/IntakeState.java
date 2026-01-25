package frc.robot.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class IntakeState extends VirtualSubsystem {
    protected static final String LogKey = "IntakeState";

    private final Intake intake;

    public enum State {
        NONE,
        EJECTING,
        INTAKING
    }

    private State intakeState = State.NONE;

    public final Trigger isNone = isStateTrigger(() -> intakeState, State.NONE);
    public final Trigger isEjecting = isStateTrigger(() -> intakeState, State.EJECTING);
    public final Trigger isIntaking = isStateTrigger(() -> intakeState, State.INTAKING);

    public IntakeState(
            final Constants.RobotMode mode,
            final Intake intake
    ) {
        this.intake = intake;

        configureStateTriggers();

        if (mode != Constants.RobotMode.REAL) {
            configureSimTriggers();
        }
    }

    public Trigger isStateTrigger(final Supplier<State> currentState, final State state) {
        return new Trigger(() -> currentState.get() == state);
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/IntakeState", intakeState);

        Logger.recordOutput("/IsNone", isNone.getAsBoolean());
        Logger.recordOutput("/IsEjecting", isEjecting.getAsBoolean());
        Logger.recordOutput("/IsIntaking", isIntaking.getAsBoolean());
    }

    public Command setState(final State state) {
        return Commands.runOnce(() -> this.intakeState = state)
                .withName("SetIntakeState: " + state.toString());
    }

    //TODO: Finish state triggers and sim triggers
    public void configureStateTriggers() {

    }

    public void configureSimTriggers() {

    }
}
