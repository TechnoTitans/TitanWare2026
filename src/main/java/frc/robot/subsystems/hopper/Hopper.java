package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    protected static final String LogKey = "Hopper";
    private static final double VelocityToleranceRotsPerSec = 0.002;

    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOP;
    private Goal currentGoal = desiredGoal;

    public enum Goal {
        STOP(0),
        INTAKE(4),
        FEED(5),
        EJECT(-5);

        private final double rollerVelocitySetpoint;

        Goal(final double rollerVelocitySetpoint) {
            this.rollerVelocitySetpoint = rollerVelocitySetpoint;
        }

        public double getRollerVelocitySetpoint() {
            return rollerVelocitySetpoint;
        }
    }

    public Hopper(final Constants.RobotMode mode, final HardwareConstants.HopperConstants constants) {
        this.hopperIO = switch (mode) {
            case REAL -> new HopperIOReal(constants);
            case SIM -> new HopperIOSim(constants);
            case DISABLED, REPLAY -> new HopperIO() {};
        };

        this.inputs = new HopperIOInputsAutoLogged();

        hopperIO.config();

        hopperIO.toRollerVelocity(desiredGoal.getRollerVelocitySetpoint());
    }

    @Override
    public void periodic() {
        final double HopperPeriodicFPGATime = Timer.getFPGATimestamp();

        hopperIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            hopperIO.toRollerVelocity(desiredGoal.getRollerVelocitySetpoint());
            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/HopperVelocityRotsPerSec", desiredGoal.getRollerVelocitySetpoint());
        Logger.recordOutput(LogKey + "/Triggers/AtVelocitySetpoint", atVelocitySetpoint());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - HopperPeriodicFPGATime)
        );
    }

    public Command toGoal(final Goal desiredGoal) {
        return runEnd(
                () -> setGoal(desiredGoal),
                () -> setGoal(Goal.STOP)
        ).withName("ToGoal");
    }

    public void setGoal(final Goal desiredGoal) {
        this.desiredGoal = desiredGoal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    public boolean atVelocitySetpoint() {
        return MathUtil.isNear(desiredGoal.rollerVelocitySetpoint, inputs.rollerVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }
}
