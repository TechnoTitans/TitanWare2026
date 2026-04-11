package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SimConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.params.MovingUtils;
import frc.robot.utils.Container;
import frc.robot.utils.commands.ext.CommandsExt;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.ThreadLocalRandom;
import java.util.function.Function;

public class FuelState extends VirtualSubsystem {
    protected static final String LogKey = "FuelState";
    private final Trigger teleopEnabled = RobotModeTriggers.teleop();

    private final DeltaTime deltaTime;
    private final Constants.RobotMode mode;

    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final Superstructure superstructure;
    private final ComponentsSolver componentsSolver;

    private int simFuelCount = 0;
    private final FuelCache fuelCache;
    private final Trigger hasSimFuel;

    private int simScoredFuelCount = 0;
    private double simTimeOfFlight;

    public final Trigger hasFuel;

    public FuelState(
            final Constants.RobotMode mode,
            final Swerve swerve,
            final Intake intake,
            final Indexer indexer,
            final Superstructure superstructure,
            final ComponentsSolver componentsSolver
    ) {
        this.deltaTime = new DeltaTime(true);
        this.mode = mode;

        this.swerve = swerve;
        this.intake = intake;
        this.indexer = indexer;
        this.superstructure = superstructure;
        this.componentsSolver = componentsSolver;

        this.hasSimFuel = new Trigger(() -> simFuelCount > 0);
        // TODO: check tof sensor
//        this.hasFuel = group.t("HasFuel", indexer::isFeederTOFDetected)
//                .debounce(0.5, Debouncer.DebounceType.kFalling);
        this.hasFuel = new Trigger(() -> true);

        configureStateTriggers();
        switch (mode) {
            case SIM, REPLAY -> {
                this.fuelCache = new FuelCache(50, fuel -> {
                    final Pose2d hubPose = FieldConstants.getHubPose();
                    if (isInsideHub(hubPose, fuel)) {
                        simScoredFuelCount++;
                        simTimeOfFlight = fuel.getTimeOfFlightSeconds();

                        return true;
                    }

                    return false;
                });

                configureSimTriggers();
                setSimFuelCount(simFuelCount);
            }
            default -> this.fuelCache = null;
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/HasFuel", hasFuel);

        switch (mode) {
            case SIM, REPLAY -> {
                fuelCache.periodic(deltaTime.get());
                Logger.recordOutput(LogKey + "/SimFuelPoses", fuelCache.getPoses());

                Logger.recordOutput(LogKey + "/SimFuelCount", simFuelCount);
                Logger.recordOutput(LogKey + "/HasSimFuel", hasSimFuel);
                Logger.recordOutput(LogKey + "/SimScoredFuelCount", simScoredFuelCount);
                Logger.recordOutput(LogKey + "/SimTimeOfFlight", simTimeOfFlight);
            }
        }
    }

    public void setSimFuelCount(final int simFuelCount) {
        switch (mode) {
            case SIM, REPLAY -> {
                this.simFuelCount = simFuelCount;
                if (simFuelCount > 0) {
                    indexer.setFeederTOFDetected(true);
                }
            }
        }
    }

    public void setSimFuelPreloaded() {
        setSimFuelCount(8);
    }

    private void configureStateTriggers() {
        intake.isIntaking.and(hasFuel.negate())
                .whileTrue(CommandsExt.defaultCommand(indexer.feed()));
    }

    private void configureSimTriggers() {
        final ThreadLocalRandom random = ThreadLocalRandom.current();

        teleopEnabled.onTrue(Commands.runOnce(() -> setSimFuelCount(500)));

        final double fuelIntakePerSecond = 10;
        intake.isIntaking.whileTrue(setInterval(1 / fuelIntakePerSecond, () -> simFuelCount++));

        final double fuelFedPerSecond = 18;
        indexer.isIndexing
                .and(hasFuel)
                .and(hasSimFuel)
                .whileTrue(setInterval(
                        1 / fuelFedPerSecond,
                        () -> {
                            final Pose2d robotPose = swerve.getPose();
                            final Pose3d hoodComponentPose = componentsSolver.getSuperstructurePoses()[1];
                            final Pose3d hoodPose = new Pose3d(robotPose)
                                    .plus(new Transform3d(
                                            hoodComponentPose.getTranslation(),
                                            hoodComponentPose.getRotation()
                                    ))
                                    .plus(SimConstants.Hood.FuelExitOffset);

                            final ChassisSpeeds turretFieldSpeeds = MovingUtils.getTurretFieldSpeeds(
                                    robotPose,
                                    superstructure.getTurretTranslation(robotPose),
                                    swerve.getFieldRelativeSpeeds()
                            );

                            fuelCache.spawn(
                                    hoodPose,
                                    ShooterOmegaToBallVelocity.get(superstructure.getShooterVelocityRotsPerSec()),
                                    turretFieldSpeeds
                            );
                            simFuelCount = Math.max(simFuelCount - 1, 0);
                        }
                ));

        intake.isIntaking
                .and(indexer.isIndexing)
                .and(hasFuel.negate())
                .and(hasSimFuel)
                .onTrue(Commands.sequence(
                        waitRand(random, 0.25, 0.5),
                        Commands.runOnce(() -> indexer.setFeederTOFDetected(true))
                ));
        indexer.isIndexing
                .and(hasSimFuel.negate())
                .onTrue(Commands.runOnce(() -> indexer.setFeederTOFDetected(false)));
    }

    @SuppressWarnings("SameParameterValue")
    private static Command waitRand(
            final ThreadLocalRandom random,
            final double lowerInclusiveSeconds,
            final double upperExclusiveSeconds
    ) {
        return Commands.waitSeconds(random.nextDouble(lowerInclusiveSeconds, upperExclusiveSeconds));
    }

    private static Command setInterval(final double intervalSeconds, final Runnable callback) {
        final Container<Double> tContainer = Container.of(0d);
        final DeltaTime deltaTime = new DeltaTime();

        return Commands.runEnd(
                () -> {
                    double t = tContainer.get();
                    t += deltaTime.get();

                    while (t >= intervalSeconds) {
                        t -= intervalSeconds;
                        callback.run();
                    }
                    tContainer.set(t);
                },
                deltaTime::reset
        ).withName("SetInterval");
    }

    private static final double HubRadiusMeters = 0.615;
    private static final double HubHeightMeters = 1.829;
    private static boolean isInsideHub(
            final Pose2d hubPose,
            final FuelCache.Fuel fuel
    ) {
        final double z = fuel.getZ();
        final double lowZ = HubHeightMeters - Units.inchesToMeters(4);
        return (Math.hypot(hubPose.getX() - fuel.getX(), hubPose.getY() - fuel.getY()) <= HubRadiusMeters)
                && (z >= lowZ && z <= HubHeightMeters)
                && (fuel.vz < 0);
    }

    private static class FuelCache {
        private static final Translation3d FAR_AWAY =
                new Translation3d(-100, -100, -100);

        private final Fuel[] fuel;
        private final Function<Fuel, Boolean> shouldDiscard;

        private int index = 0;

        public FuelCache(final int capacity, final Function<Fuel, Boolean> shouldDiscard) {
            this.fuel = new Fuel[capacity];
            for (int i = 0; i < capacity; i++) {
                this.fuel[i] = new Fuel(FAR_AWAY);
            }

            this.shouldDiscard = shouldDiscard;
        }

        public void spawn(
                final Pose3d pose,
                final double velocityMetersPerSec,
                final ChassisSpeeds turretFieldSpeeds
        ) {
            final Fuel cached = fuel[index];
            cached.at(pose, velocityMetersPerSec, turretFieldSpeeds);
            index = (index + 1) % fuel.length;
        }

        public void periodic(final double dtSeconds) {
            for (final Fuel cached : fuel) {
                if (!cached.active) {
                    continue;
                }

                cached.update(dtSeconds);
                if (cached.getZ() < 0) {
                    cached.discard(FAR_AWAY);
                }

                if (shouldDiscard != null) {
                    if (shouldDiscard.apply(cached)) {
                        cached.discard(FAR_AWAY);
                    }
                }
            }
        }

        public Pose3d[] getPoses() {
            final int fuelCount = fuel.length;
            final Pose3d[] poses = new Pose3d[fuelCount];
            for (int i = 0; i < fuel.length; i++) {
                final Fuel cached = fuel[i];
                poses[i] = cached.getPose();
            }

            return poses;
        }

        private static class Fuel {
            private static final Translation3d ForwardAxis = new Translation3d(1, 0, 0);
            private static final Vector<N3> ForwardAxisVec = ForwardAxis.toVector();
            private static final double GravityMetersPerSecSquared = 9.81;

            private boolean active = false;
            private double activeStartTime = 0;

            private double x;
            private double y;
            private double z;

            private double vx = 0.0;
            private double vy = 0.0;
            private double vz = 0.0;

            public Fuel(final Translation3d pos) {
                this.x = pos.getX();
                this.y = pos.getY();
                this.z = pos.getZ();
            }

            public void at(
                    final Pose3d pose,
                    final double velocityMetersPerSec,
                    final ChassisSpeeds turretFieldSpeeds
            ) {
                active = true;
                activeStartTime = Timer.getFPGATimestamp();

                x = pose.getX();
                y = pose.getY();
                z = pose.getZ();

                final Vector<N3> axis = ForwardAxis.rotateBy(pose.getRotation()).toVector();
                final Pose3d randomized = new Pose3d(
                        pose.getTranslation(),
                        new Rotation3d(ForwardAxisVec, randomCone(axis, Units.degreesToRadians(1)))
                );

                final Translation3d velocity = ForwardAxis.rotateBy(randomized.getRotation())
                        .times(velocityMetersPerSec);

                vx = velocity.getX() + turretFieldSpeeds.vxMetersPerSecond;
                vy = velocity.getY() + turretFieldSpeeds.vyMetersPerSecond;
                vz = velocity.getZ();
            }

            public void update(final double dtSeconds) {
                if (!active) {
                    return;
                }

                x += vx * dtSeconds;
                y += vy * dtSeconds;
                z += vz * dtSeconds
                        - 0.5 * GravityMetersPerSecSquared * dtSeconds * dtSeconds;

                vz -= GravityMetersPerSecSquared * dtSeconds;
            }

            public void discard(final Translation3d to) {
                active = false;

                x = to.getX();
                y = to.getY();
                z = to.getZ();

                vx = 0.0;
                vy = 0.0;
                vz = 0.0;
            }

            public double getTimeOfFlightSeconds() {
                return Timer.getFPGATimestamp() - activeStartTime;
            }

            public double getX() {
                return x;
            }

            public double getY() {
                return y;
            }

            public double getZ() {
                return z;
            }

            public Pose3d getPose() {
                return new Pose3d(x, y, z, Rotation3d.kZero);
            }

            private static final Vector<N3> ZAxis = VecBuilder.fill(0, 0, 1);
            public static Vector<N3> randomCone(final Vector<N3> axis, final double angleRads) {
                final double cos = Math.cos(angleRads);
                final double z = 1 - (Math.random() * (1 - cos));
                final double phi = 2 * Math.PI * Math.random();
                final double r = Math.sqrt(1 - (z * z));
                final double x = r * Math.cos(phi);
                final double y = r * Math.sin(phi);
                final Vector<N3> vec = VecBuilder.fill(x, y, z);

                if (z >= 1) {
                    return vec;
                } else if (z <= -1) {
                    return vec.times(-1);
                }

                final Vector<N3> dir = Vector.cross(ZAxis, axis);
                final double rot = Math.acos(axis.dot(ZAxis));

                final Matrix<N3, N1> rVec = new Rotation3d(dir, rot).toMatrix().times(vec);
                return VecBuilder.fill(rVec.get(0, 0), rVec.get(1, 0), rVec.get(2, 0));
            }
        }
    }

    private static final InterpolatingDoubleTreeMap ShooterOmegaToBallVelocity = new InterpolatingDoubleTreeMap();
    private static double shooterSurfaceVelocity(final double shooterVelocityRotsPerSec) {
        return shooterVelocityRotsPerSec * SimConstants.Shooter.WHEEL_CIRCUMFERENCE_METERS;
    }
    static {
        ShooterOmegaToBallVelocity.put(0d, 0d);
        ShooterOmegaToBallVelocity.put(10d, shooterSurfaceVelocity(10d));
        ShooterOmegaToBallVelocity.put(20d, shooterSurfaceVelocity(20d));
        ShooterOmegaToBallVelocity.put(30d, shooterSurfaceVelocity(30d));
        ShooterOmegaToBallVelocity.put(40d, shooterSurfaceVelocity(40d));
        ShooterOmegaToBallVelocity.put(50d, shooterSurfaceVelocity(50d));
    }
}