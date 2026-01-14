package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    protected static final String LogKey = "Shooter";
    private static final double VelocityToleranceRotsPerSec = 0.2;

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.NONE;
    private Goal currentGoal = desiredGoal;

    public final Trigger atVelocitySetpoint = new Trigger(this::atVelocitySetpoint);

    public enum Goal {
        NONE(0),
        SHOOT(10);

        private final double setpoint;

        Goal(final double setpoint) {
            this.setpoint = setpoint;
        }

        public double getSetpoint() {
            return setpoint;
        }
    }
    public Shooter(final Constants.RobotMode mode, final HardwareConstants.ShooterConstants constants) {
        this.shooterIO = switch (mode) {
            case REAL -> new ShooterIOReal(constants);
            case SIM -> new ShooterIOSim(constants);
            case REPLAY, DISABLED -> new ShooterIO() {
            };
        };

        this.inputs = new ShooterIOInputsAutoLogged();

        this.shooterIO.config();
        this.shooterIO.toFlywheelVelocity(desiredGoal.setpoint);
    }

    @Override
    public void periodic() {
        final double ShooterPeriodicUpdateStart = Timer.getFPGATimestamp();

        shooterIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            shooterIO.toFlywheelVelocity(desiredGoal.setpoint);
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/ShooterVelocityRotsPerSec", desiredGoal.getSetpoint());

        Logger.recordOutput(LogKey + "/Triggers/AtVelocitySetpoint", atVelocitySetpoint());
        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - ShooterPeriodicUpdateStart)
        );
    }

    public void setDesiredGoal(final Goal desiredGoal) {
        this.desiredGoal = desiredGoal;
    }

    public Command toGoal(final Goal desiredGoalSupplier) {
        return runEnd(
                () -> setDesiredGoal(desiredGoalSupplier),
                () -> setDesiredGoal(Goal.NONE)
        ).withName("ToGoal");
    }

    public Command setGoal(final Goal desiredGoal) {
        return Commands.runOnce(
                () -> {
                    this.desiredGoal = desiredGoal;
                    Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
                    Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
                }
        );
    }

    private boolean atVelocitySetpoint() {
        return MathUtil.isNear(
                desiredGoal.getSetpoint(),
                inputs.flywheelVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        );
    }
}
