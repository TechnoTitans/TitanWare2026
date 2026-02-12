package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.slide.IntakeSlide;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.teleop.SwerveSpeed;
import org.littletonrobotics.junction.Logger;

public class RobotCommands {
    public enum ScoringMode {
        Stationary,
        Moving,
        Turret_Off
    }

    protected static final String LogKey = "RobotCommands";
    protected static final double AllowableSpeedToShootMetersPerSec = 0.1;

    private final Swerve swerve;
    private final IntakeRoller intakeRoller;
    private final IntakeSlide intakeSlide;
    private final Superstructure superstructure;
    private final Spindexer spindexer;
    private final Feeder feeder;

    private final Trigger ableToShoot;

    public RobotCommands(
            final Swerve swerve,
            final IntakeRoller intakeRoller,
            final IntakeSlide intakeSlide,
            final Superstructure superstructure,
            final Spindexer spindexer,
            final Feeder feeder
    ) {
        this.swerve = swerve;
        this.intakeRoller = intakeRoller;
        this.intakeSlide = intakeSlide;
        this.superstructure = superstructure;
        this.spindexer = spindexer;
        this.feeder = feeder;

        this.ableToShoot = new Trigger(() -> {
            final ChassisSpeeds swerveChassisSpeed = swerve.getRobotRelativeSpeeds();

            return Math.hypot(
                    swerveChassisSpeed.vxMetersPerSecond,
                    swerveChassisSpeed.vyMetersPerSecond
            ) < AllowableSpeedToShootMetersPerSec;
        });
    }

    public void periodic() {
        Logger.recordOutput(LogKey + "/AbleToShoot", ableToShoot);
    }

    public Command manualIntake() {
        return Commands.parallel(
                intakeRoller.setGoal(IntakeRoller.Goal.INTAKE),
                intakeSlide.setGoal(IntakeSlide.Goal.INTAKE)
        ).withName("ManualIntake");
    }

    public Command stowIntake() {
        return Commands.parallel(
                intakeSlide.setGoal(IntakeSlide.Goal.STOW),
                intakeRoller.setGoal(IntakeRoller.Goal.STOW)
        ).withName("StowIntake");
    }

    //TODO: Might need to change the feeding
    public Command shootWhileMoving() {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.SHOOTING),
                Commands.repeatingSequence(
                        Commands.waitUntil(superstructure.atSuperstructureSetpoint),
                        feeder.toGoal(Feeder.Goal.FEED)
                                .until(superstructure.atSuperstructureSetpoint.negate())
                ),
                spindexer.toGoal(Spindexer.Goal.FEED)
                        .onlyIf(superstructure.atSuperstructureSetpoint),
                        Commands.runOnce(() -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.SHOOTING))
                )
                .finallyDo(() -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.NORMAL))
                .withName("ShootWhileMoving");
    }

    public Command shootStationary() {
        return Commands.parallel(
                Commands.sequence(
                        Commands.waitUntil(ableToShoot),
                        Commands.parallel(
                                superstructure.toGoal(Superstructure.Goal.SHOOTING),
                                Commands.repeatingSequence(
                                        Commands.waitUntil(superstructure.atSuperstructureSetpoint),
                                        feeder.toGoal(Feeder.Goal.FEED)
                                                .until(superstructure.atSuperstructureSetpoint.negate())
                                )
                        )
                ),
                spindexer.toGoal(Spindexer.Goal.FEED)
                        .onlyIf(ableToShoot.and(superstructure.atSuperstructureSetpoint)),
                swerve.runWheelXCommand()
        ).withName("ShootStationary");
    }
}