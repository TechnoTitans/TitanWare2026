package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.slide.IntakeSlide;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class RobotCommands {
    protected static final String LogKey = "RobotCommands";
    protected static final double AllowableSpeedToShootMetersPerSec = 0.1;

    private final Swerve swerve;
    private final IntakeRoller intakeRoller;
    private final IntakeSlide intakeSlide;
    private final Superstructure superstructure;
    private final Spindexer spindexer;

    private final Trigger ableToShoot;

    public RobotCommands(
            final Swerve swerve,
            final IntakeRoller intakeRoller,
            final IntakeSlide intakeSlide,
            final Superstructure superstructure,
            final Spindexer spindexer
    ) {
        this.swerve = swerve;
        this.intakeRoller = intakeRoller;
        this.intakeSlide = intakeSlide;
        this.superstructure = superstructure;
        this.spindexer = spindexer;

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
                intakeRoller.toGoal(IntakeRoller.Goal.INTAKE),
                intakeSlide.setGoal(IntakeSlide.Goal.INTAKE),
                spindexer.toGoal(Spindexer.Goal.INTAKE)
        ).withName("Intake");
    }

    public Command shootWhileMoving() {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.SHOOTING),
                spindexer.toGoal(Spindexer.Goal.FEED),
                intakeSlide.toGoal(IntakeSlide.Goal.SHOOTING)
        ).withName("ShootWhileMoving");
    }

    public Command shootStationary(final Supplier<Rotation2d> rotation2dSupplier) {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(ableToShoot),
                        superstructure.toGoal(Superstructure.Goal.SHOOTING)
                ),
                swerve.faceAngle(rotation2dSupplier),
                spindexer.toGoal(Spindexer.Goal.FEED).onlyIf(ableToShoot),
                intakeSlide.toGoal(IntakeSlide.Goal.SHOOTING).onlyIf(ableToShoot)
        ).withName("ShootStationary");
    }
}