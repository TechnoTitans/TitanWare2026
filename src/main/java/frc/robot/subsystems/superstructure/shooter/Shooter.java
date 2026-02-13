package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    protected static final String LogKey = "Superstructure/Shooter";
    private static final double VelocityToleranceRotsPerSec = 0.2;

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOP;
    private Goal currentGoal = desiredGoal;

    public final Trigger atVelocitySetpoint = new Trigger(this::atVelocitySetpoint);

    public enum Goal {
        STOP(0, false),
        TRACKING(0, true);

        private double velocitySetpoint;
        private final boolean isDynamic;

        Goal(final double setpoint, final boolean isDynamic) {
            this.velocitySetpoint = setpoint;
            this.isDynamic = isDynamic;
        }

        public double getVelocitySetpoint() {
            return velocitySetpoint;
        }

        public void changeShooterVelocityGoal(final double desiredShooterVelocity) {
            if (isDynamic) {
                this.velocitySetpoint = desiredShooterVelocity;
            }
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
        this.shooterIO.toVelocity(desiredGoal.velocitySetpoint);
    }

    @Override
    public void periodic() {
        final double ShooterPeriodicUpdateStart = Timer.getFPGATimestamp();

        shooterIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal.isDynamic) {
            shooterIO.toVelocity(desiredGoal.velocitySetpoint);
        }

        if (desiredGoal != currentGoal) {
            shooterIO.toVelocity(desiredGoal.velocitySetpoint);
            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/ShooterVelocityRotsPerSec", desiredGoal.getVelocitySetpoint());

        Logger.recordOutput(LogKey + "/Triggers/AtVelocitySetpoint", atVelocitySetpoint());
        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - ShooterPeriodicUpdateStart)
        );
    }

    public boolean isShooting() {
        return atVelocitySetpoint.getAsBoolean() && currentGoal == Goal.TRACKING;
    }

    public void setGoal(final Goal desiredGoal) {
        this.desiredGoal = desiredGoal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    public void updateVelocitySetpoint(final double desiredShooterVelocity) {
        this.desiredGoal.changeShooterVelocityGoal(desiredShooterVelocity);
    }

    private boolean atVelocitySetpoint() {
        return MathUtil.isNear(
                desiredGoal.getVelocitySetpoint(),
                inputs.masterVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        );
    }
}
