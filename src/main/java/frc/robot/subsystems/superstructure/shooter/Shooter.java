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
    private static final double VelocityToleranceRotsPerSec = 2;

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOP;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSetpoint = new Trigger(this::atSetpoint);

    public enum Goal {
        STOP(0, false),
        IDLE(20, false),
        TRACKING(0, true);

        private double velocitySetpointRotsPerSec;
        private final boolean isDynamic;

        Goal(final double velocitySetpointRotsPerSec, final boolean isDynamic) {
            this.velocitySetpointRotsPerSec = velocitySetpointRotsPerSec;
            this.isDynamic = isDynamic;
        }

        public void changeShooterVelocityGoal(final double desiredShooterVelocity) {
            if (isDynamic) {
                this.velocitySetpointRotsPerSec = desiredShooterVelocity;
            }
        }
    }

    public Shooter(final Constants.RobotMode mode, final HardwareConstants.ShooterConstants constants) {
        this.shooterIO = switch (mode) {
            case REAL -> new ShooterIOReal(constants);
            case SIM -> new ShooterIOSim(constants);
            case REPLAY, DISABLED -> new ShooterIO() {};
        };

        this.inputs = new ShooterIOInputsAutoLogged();

        this.shooterIO.config();
        this.shooterIO.toVelocity(desiredGoal.velocitySetpointRotsPerSec);
    }

    @Override
    public void periodic() {
        final double shooterPeriodicUpdateStart = Timer.getFPGATimestamp();

        shooterIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            currentGoal = desiredGoal;
            shooterIO.toVelocity(desiredGoal.velocitySetpointRotsPerSec);
        } else if (desiredGoal.isDynamic) {
            shooterIO.toVelocity(desiredGoal.velocitySetpointRotsPerSec);
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/VelocitySetpointRotsPerSec", desiredGoal.velocitySetpointRotsPerSec);

        Logger.recordOutput(LogKey + "/Triggers/AtSetpoint", atSetpoint());
        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - shooterPeriodicUpdateStart)
        );
    }

    public Command setGoalCommand(final Goal goal) {
        return Commands.runOnce(() -> setGoal(goal));
    }

    public double getVelocityRotsPerSec() {
        return inputs.masterVelocityRotsPerSec;
    }

    public void setGoal(final Goal goal) {
        desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", goal.toString());
    }

    public void updateVelocitySetpoint(final double desiredShooterVelocity) {
        desiredGoal.changeShooterVelocityGoal(desiredShooterVelocity);
    }

    private boolean atSetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(
                desiredGoal.velocitySetpointRotsPerSec,
                inputs.masterVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        );
    }
}
