package frc.robot.sim.fuel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.SimConstants;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class FuelSimManager {
    private static final LinearVelocity BALL_SPEED = MetersPerSecond.of(7.5);

    private final Timer timer;
    private final FuelSim fuelSim;

    private final Supplier<Rotation2d> hoodAngleSupplier;
    private final Supplier<Rotation2d> turretYawSupplier;
    private final Supplier<Pose2d> swervePoseSupplier;
    private final Supplier<ChassisSpeeds> swerveSpeedsSupplier;
    private final BooleanSupplier isIntaking;
    private final Trigger isShooting;

    public FuelSimManager(
            Supplier<Rotation2d> hoodAngleSupplier,
            Supplier<Rotation2d> turretYawSupplier,
            Supplier<Pose2d> swervePoseSupplier,
            Supplier<ChassisSpeeds> swerveSpeedsSupplier,
            BooleanSupplier isIntaking,
            BooleanSupplier isShooting
    ) {
        this.fuelSim = new FuelSim();
        this.timer = new Timer();

        this.hoodAngleSupplier = hoodAngleSupplier;
        this.turretYawSupplier = turretYawSupplier;
        this.swervePoseSupplier = swervePoseSupplier;
        this.swerveSpeedsSupplier = swerveSpeedsSupplier;
        this.isIntaking = isIntaking;
        this.isShooting = new Trigger(isShooting);

        this.isShooting.onTrue(Commands.runOnce(timer::reset));
        this.isShooting.onFalse(Commands.runOnce(fuelSim::clearFuel));

        timer.start();
        fuelSim.start();

        configureFuelSimRobot();
    }

    public void periodic() {
        if (isShooting.getAsBoolean()) {
            if (timer.advanceIfElapsed(0.25)) {
                fuelSim.launchFuel(
                        BALL_SPEED,
                        (Rotation2d.kCCW_90deg.minus(hoodAngleSupplier.get())).getMeasure(),
                        turretYawSupplier.get().getMeasure(),
                        SimConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_3D
                );
            }
        }

        fuelSim.updateSim();
    }



    private void configureFuelSimRobot() {
        // Register a robot for collision with fuel
        fuelSim.registerRobot(
                0.533, // from left to right in meters
                0.680, // from front to back in meters
                0.127, // from floor to top of bumpers in meters
                swervePoseSupplier, // Supplier<Pose2d> of robot pose
                swerveSpeedsSupplier); // Supplier<ChassisSpeeds> of field-centric chassis speeds
        // Register an intake to remove fuel from the field as a rectangular bounding box
    }
}
