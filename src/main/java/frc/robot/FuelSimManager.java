package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.SimConstants;
import frc.robot.utils.FuelSim;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class FuelSimManager {
    private static final int CAPACITY = 50;
    private static final LinearVelocity BALL_SPEED = MetersPerSecond.of(5.35);
    private static final Distance LAUNCH_HEIGHT = Inches.of(15.2);

    private FuelSim fuelSim;
    private int fuelStored = 0;

    private final Supplier<Rotation2d> hoodAngleSupplier;
    private final Supplier<Rotation2d> turretYawSupplier;
    private final Supplier<Pose2d> swervePoseSupplier;
    private final Supplier<ChassisSpeeds> swerveSpeedsSupplier;
    private final BooleanSupplier isIntaking;
    private final BooleanSupplier isShooting;

    public FuelSimManager(
            Supplier<Rotation2d> hoodAngleSupplier,
            Supplier<Rotation2d> turretYawSupplier,
            Supplier<Pose2d> swervePoseSupplier,
            Supplier<ChassisSpeeds> swerveSpeedsSupplier,
            BooleanSupplier isIntaking,
            BooleanSupplier isShooting
    ) {
        this.fuelSim = new FuelSim();

        this.hoodAngleSupplier = hoodAngleSupplier;
        this.turretYawSupplier = turretYawSupplier;
        this.swervePoseSupplier = swervePoseSupplier;
        this.swerveSpeedsSupplier = swerveSpeedsSupplier;
        this.isIntaking = isIntaking;
        this.isShooting = isShooting;

        configureFuelSim();
        configureFuelSimRobot();
    }

    public void periodic() {
        if (isShooting.getAsBoolean()) {
            if (fuelStored == 0)
                return;

            fuelStored--;

            fuelSim.launchFuel(
                    BALL_SPEED,
                    Rotation2d.fromDegrees(45).getMeasure(),
                    turretYawSupplier.get().getMeasure(),
                    SimConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_3D
            );
        }

        fuelSim.stepSim();
        fuelSim.updateSim();
    }

    private boolean canIntake() {
        return fuelStored < CAPACITY;
    }

    private void intakeFuel() {
        fuelStored++;
    }

    private void configureFuelSim() {
        fuelSim.spawnStartingFuel(); // spawns fuel in the depots and neutral zone
        fuelSim.enableAirResistance();

        fuelSim.start();
        SmartDashboard.putData(Commands.runOnce(() -> {
                    fuelSim.clearFuel();
                    fuelSim.spawnStartingFuel();
                })
                .withName("Reset Fuel")
                .ignoringDisable(true));
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

        fuelSim.registerIntake(
                Units.inchesToMeters(19), Units.inchesToMeters(25), Units.inchesToMeters(-16.5), Units.inchesToMeters(16.5), // robot-centric coordinates for bounding box in meters
                () -> isIntaking.getAsBoolean() && canIntake(), // (optional) BooleanSupplier for whether the intake should be active at a given moment
                this::intakeFuel); // (optional) Runnable called whenever a fuel is intaked
    }
}
