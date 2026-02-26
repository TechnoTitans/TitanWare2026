package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.slide.IntakeSlide;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.subsystems.VirtualSubsystem;
import frc.robot.utils.teleop.SwerveSpeed;
import org.littletonrobotics.junction.Logger;

public class RobotCommands extends VirtualSubsystem {
    protected static final String LogKey = "RobotCommands";
    protected static final double AllowableSpeedToShootMetersPerSec = 0.1;

    private final Swerve swerve;
    private final IntakeRoller intakeRoller;
    private final IntakeSlide intakeSlide;
    private final Superstructure superstructure;
    private final Spindexer spindexer;
    private final Feeder feeder;
    private final Climb climb;

    private final Trigger ableToShoot;

    public enum ScoringMode {
        Stationary,
        Moving
    }

    public RobotCommands(
            final Swerve swerve,
            final IntakeRoller intakeRoller,
            final IntakeSlide intakeSlide,
            final Superstructure superstructure,
            final Spindexer spindexer,
            final Feeder feeder,
            final Climb climb
    ) {
        this.swerve = swerve;
        this.intakeRoller = intakeRoller;
        this.intakeSlide = intakeSlide;
        this.superstructure = superstructure;
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.climb = climb;

        this.ableToShoot = new Trigger(() -> {
            final ChassisSpeeds swerveChassisSpeed = swerve.getRobotRelativeSpeeds();

            return Math.hypot(
                    swerveChassisSpeed.vxMetersPerSecond,
                    swerveChassisSpeed.vyMetersPerSecond
            ) < AllowableSpeedToShootMetersPerSec;
        });
    }

    //TODO: Make logged trigger
    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/AbletoShoot", ableToShoot);
    }

    public Command deployIntake() {
        return Commands.parallel(
                intakeRoller.setGoal(IntakeRoller.Goal.INTAKE),
                intakeSlide.setGoal(IntakeSlide.Goal.INTAKE)
        ).withName("DeployIntake");
    }

    public Command stowIntake() {
        return Commands.parallel(
                intakeSlide.setGoal(IntakeSlide.Goal.STOW),
                intakeRoller.setGoal(IntakeRoller.Goal.STOP)
        ).withName("StowIntake");
    }

    public Command shootWhileMoving() {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.SHOOTING),
                Commands.repeatingSequence(
                        Commands.waitUntil(superstructure.atSetpoint),
                        feeder.toGoal(Feeder.Goal.FEED)
                                .onlyWhile(superstructure.atSetpoint)
                ),
                intakeSlide.toGoal(IntakeSlide.Goal.SHOOTING),
                spindexer.toGoal(Spindexer.Goal.FEED),
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
                                        Commands.waitUntil(superstructure.atSetpoint),
                                        feeder.toGoal(Feeder.Goal.FEED)
                                                .until(superstructure.atSetpoint.negate())
                                )
                        )
                ),
                intakeSlide.toGoal(IntakeSlide.Goal.SHOOTING),
                spindexer.toGoal(Spindexer.Goal.FEED),
                swerve.runWheelXCommand()
        ).withName("ShootStationary");
    }

    public Command readyClimb() {
        return Commands.parallel(
                swerve.runToPose(FieldConstants::getClimbTarget),
                superstructure.setGoal(Superstructure.Goal.CLIMB),
                Commands.sequence(
                        stowIntake(),
                        Commands.waitUntil(intakeSlide.atSlideSetpoint),
                        climb.setGoal(Climb.Goal.EXTEND)
                )
        ).withName("ReadyClimb");
    }

    public Command climb() {
        return Commands.parallel(
                swerve.wheelXCommand(),
                climb.setGoal(Climb.Goal.CLIMB_DOWN)
        ).withName("Climb");
    }

    public Command unclimb() {
        return Commands.parallel(
                superstructure.setGoal(Superstructure.Goal.TRACKING),
                climb.setGoal(Climb.Goal.STOW)
        ).withName("Unclimb");
    }
}