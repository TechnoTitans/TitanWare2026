package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotCommands {
    private final Swerve swerve;
    private final Intake intake;
    private final Superstructure superstructure;
    private final Hopper hopper;

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
    }

    public Command intake() {
        return Commands.parallel(
                intake.toGoal(Intake.Goal.INTAKE),
                hopper.toGoal(Hopper.Goal.INTAKE)
        );
    }
}