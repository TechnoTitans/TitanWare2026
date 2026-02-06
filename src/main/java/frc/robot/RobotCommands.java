package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.slide.IntakeSlide;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.superstructure.ShotCalculator;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.Map;
import java.util.function.Supplier;

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
    private final Climb climb;

    private final Trigger ableToShoot;

    public RobotCommands(
            final Swerve swerve,
            final IntakeRoller intakeRoller,
            final IntakeSlide intakeSlide,
            final Superstructure superstructure,
            final Spindexer spindexer,
            final Climb climb
    ) {
        this.swerve = swerve;
        this.intakeRoller = intakeRoller;
        this.intakeSlide = intakeSlide;
        this.superstructure = superstructure;
        this.spindexer = spindexer;
        this.climb = climb;

        this.ableToShoot = new Trigger(() -> {
            final ChassisSpeeds swerveChassisSpeed = swerve.getRobotRelativeSpeeds();

            return Math.hypot(swerveChassisSpeed.vxMetersPerSecond, swerveChassisSpeed.vyMetersPerSecond) < AllowableSpeedToShootMetersPerSec;
        });
    }

    public void periodic() {
        Logger.recordOutput(LogKey + "/AbleToShoot", ableToShoot);
    }

    public Command manualIntake() {
        return Commands.parallel(
                intakeRoller.toGoal(IntakeRoller.Goal.INTAKE),
                intakeSlide.setGoal(IntakeSlide.Goal.INTAKE),
                spindexer.toGoal(Spindexer.Goal.INTAKE)
        ).withName("Intake");
    }

    public Command shoot(final Supplier<ScoringMode> scoringType, final Supplier<ShotCalculator.Target> target) {
        return Commands.select(
                Map.of(
                        ScoringMode.Moving,
                        shootWhileMoving(),
                        ScoringMode.Stationary,
                        shootStationary(),
                        ScoringMode.Turret_Off,
                        alignSwerveAndShoot(() -> target.get().getTargetTranslation().minus(swerve.getPose().getTranslation()).getAngle())
                ),
                scoringType
        );
    }

    private Command shootWhileMoving() {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.SHOOTING),
                spindexer.toGoal(Spindexer.Goal.FEED)
                        .onlyIf(superstructure.atSuperstructureSetpoint)
        ).withName("ShootWhileMoving");
    }

    private Command shootStationary() {
        return Commands.parallel(
                Commands.sequence(
                        Commands.waitUntil(ableToShoot),
                        superstructure.toGoal(Superstructure.Goal.SHOOTING)
                ),
                spindexer.toGoal(Spindexer.Goal.FEED)
                        .onlyIf(ableToShoot.and(superstructure.atSuperstructureSetpoint))
        );
    }

    private Command alignSwerveAndShoot(final Supplier<Rotation2d> rotation2dSupplier) {
        return Commands.parallel(
                Commands.sequence(
                        Commands.waitUntil(ableToShoot),
                        superstructure.toGoal(Superstructure.Goal.SHOOTING)
                ),
                swerve.faceAngle(rotation2dSupplier),
                spindexer.toGoal(Spindexer.Goal.FEED)
                        .onlyIf(ableToShoot.and(superstructure.atSuperstructureSetpoint))
        ).withName("ShootStationary");
    }

    public Command climb() {
        return Commands.parallel(
                superstructure.setGoal(Superstructure.Goal.CLIMB),
                climb.toGoal(Climb.Goal.EXTEND)
        ).withName("Climb");
    }
}