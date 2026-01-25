package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class RobotCommands {
    protected static final String LogKey = "RobotCommands";
    protected static final double AllowableSpeedToShootMetersPerSec = 0.1;

    private final Swerve swerve;
    private final Intake intake;
    private final Superstructure superstructure;
    private final Hopper hopper;

    private final Trigger ableToShoot;

    public RobotCommands(
            final Swerve swerve,
            final Intake intake,
            final Superstructure superstructure,
            final Hopper hopper
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.superstructure = superstructure;
        this.hopper = hopper;

        this.ableToShoot = new Trigger(() -> {
            final ChassisSpeeds swerveChassisSpeed = swerve.getRobotRelativeSpeeds();

            return Math.hypot(swerveChassisSpeed.vxMetersPerSecond, swerveChassisSpeed.vyMetersPerSecond) < AllowableSpeedToShootMetersPerSec;
        });
    }

    public void periodic() {
        Logger.recordOutput(LogKey + "/AbleToShoot", ableToShoot);
    }

    public Command intake() {
        return Commands.parallel(
                intake.toGoal(Intake.Goal.INTAKE),
                hopper.toGoal(Hopper.Goal.INTAKE)
        ).withName("Intake");
    }

    public Command shootWhileMoving() {
        return Commands.deadline(
                superstructure.toGoal(Superstructure.Goal.SHOOTING),
                hopper.toGoal(Hopper.Goal.FEED)
        ).withName("ShootWhileMoving");
    }

    public Command shootStationary(final Supplier<Rotation2d> rotation2dSupplier) {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(ableToShoot),
                        superstructure.toGoal(Superstructure.Goal.SHOOTING)
                ),
                Commands.sequence(
                        swerve.faceAngle(rotation2dSupplier)
                ),
                hopper.toGoal(Hopper.Goal.FEED).onlyIf(ableToShoot)
        ).withName("ShootStationary");
    }
}