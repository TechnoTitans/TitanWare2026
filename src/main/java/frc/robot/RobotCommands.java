package frc.robot;

import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotCommands {
    private final Swerve swerve;
    private final Intake intake;
    private final Superstructure superstructure;

    public RobotCommands(
            final Swerve swerve,
            final Intake intake,
            final Superstructure superstructure
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.superstructure = superstructure;
    }
}
