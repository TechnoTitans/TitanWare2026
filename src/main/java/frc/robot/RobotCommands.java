package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotCommands {
    private static final double AllowableSpeedToShootMetersPerSec = 0.1;

    private final Swerve swerve;
    private final Intake intake;
    private final Superstructure superstructure;

    private final Trigger ableToShoot;

    public RobotCommands(
            final Swerve swerve,
            final Intake intake,
            final Superstructure superstructure
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.superstructure = superstructure;

        this.ableToShoot = new Trigger(() -> {
            final ChassisSpeeds swerveChassisSpeed = swerve.getRobotRelativeSpeeds();

            return Math.hypot(swerveChassisSpeed.vxMetersPerSecond, swerveChassisSpeed.vyMetersPerSecond) < AllowableSpeedToShootMetersPerSec;
        });
    }
}
