package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.HashMap;
import java.util.Objects;

public class HardwareConstants {
    public static final int PowerDistributionHub = 1;

    public enum CANBus {
        RIO("rio"),
        CANIVORE("CANivore");

        private static final HashMap<String, CANBus> BusNameToCANBus = new HashMap<>();
        static {
            for (final CANBus bus : CANBus.values()) {
                BusNameToCANBus.put(bus.name, bus);
            }
        }

        public final String name;
        CANBus(final String name) {
            this.name = name;
        }

        public static CANBus fromPhoenix6CANBus(final com.ctre.phoenix6.CANBus bus) {
            return Objects.requireNonNull(
                    BusNameToCANBus.get(bus.getName()), () -> String.format("Could not get CANBus: %s", bus.getName()));
        }

        public com.ctre.phoenix6.CANBus toPhoenix6CANBus() {
            return new com.ctre.phoenix6.CANBus(name);
        }
    }

    public record IntakeRollersConstants(
            CANBus CANBus,
            int motorId,
            double gearing
    ) {}

    public static final IntakeRollersConstants INTAKE_CONSTANTS = new IntakeRollersConstants(
            CANBus.RIO,
            14,
            20.0 / 12.0
    );

    public record IntakeSlideConstants(
            CANBus CANBus,
            int masterMotorId,
            int followerMotorId,
            double averageAxisGearing,
            double differentialAxisGearing,
            double forwardLimitRots,
            double reverseLimitRots
    ) {}

    public static final IntakeSlideConstants INTAKE_SLIDE_CONSTANTS = new IntakeSlideConstants(
            CANBus.CANIVORE,
            15,
            16,
            (60.0 / 12.0) * (40.0 / 18.0),
            1,
            3.8,
            0
    );

    public record SpindexerConstants(
            CANBus CANBus,
            int motorId,
            double gearing
    ) {}

    public static final SpindexerConstants SPINDEXER_CONSTANTS = new SpindexerConstants(
            CANBus.CANIVORE,
            17,
            2
    );

    public record FeederConstants(
            CANBus CANBus,
            int motorId,
            double gearing
    ) {}

    public static final FeederConstants FEEDER_CONSTANTS = new FeederConstants(
            CANBus.CANIVORE,
            18,
            (36.0 / 12.0) * (24.0 / 18.0) * (21.0 / 45.0)
    );

    public record TurretConstants(
            CANBus CANBus,
            int motorId,
            int primaryCANcoderId,
            int secondaryCANcoderId,
            double primaryCANcoderOffsetRots,
            double secondaryCANcoderOffsetRots,
            double forwardLimitRots,
            double reverseLimitRots,
            int drivingGearTeeth,
            int drivenTurretGearTeeth,
            int primaryCANcoderGearTeeth,
            int secondaryCANcoderGearTeeth,
            double motorToGearboxGearing,
            double gearboxToTurretGearing,
            double primaryCANcoderGearing,
            double secondaryCANcoderGearing,
            Transform2d offsetFromCenter
    ) {
        public TurretConstants(
                CANBus CANBus,
                int motorId,
                int primaryCANcoderId,
                int secondaryCANcoderId,
                double primaryCANcoderOffsetRots,
                double secondaryCANcoderOffsetRots,
                double forwardLimitRots,
                double reverseLimitRots,
                int drivingGearTeeth,
                int drivenTurretGearTeeth,
                int primaryCANcoderGearTeeth,
                int secondaryCANcoderGearTeeth,
                double motorToGearboxGearing,
                Transform2d offsetFromCenter
        ) {
            this(
                    CANBus,
                    motorId,
                    primaryCANcoderId,
                    secondaryCANcoderId,
                    primaryCANcoderOffsetRots,
                    secondaryCANcoderOffsetRots,
                    forwardLimitRots,
                    reverseLimitRots,
                    drivingGearTeeth,
                    drivenTurretGearTeeth,
                    primaryCANcoderGearTeeth,
                    secondaryCANcoderGearTeeth,
                    motorToGearboxGearing,
                    ((double) drivenTurretGearTeeth) / drivingGearTeeth,
                    ((double) primaryCANcoderGearTeeth) / drivenTurretGearTeeth,
                    ((double) secondaryCANcoderGearTeeth) / drivenTurretGearTeeth,
                    offsetFromCenter
            );
        }
    }

    public static final TurretConstants TURRET_CONSTANTS = new TurretConstants(
            CANBus.CANIVORE,
            19,
            20,
            21,
            //TODO: FIND OFFSETS
            0,
            0,
            0.5,
            -0.5,
            10,
            80,
            13,
            17,
            36.0 / 12.0,
            new Transform2d(-0.127, 0, Rotation2d.kZero)
    );

    public record HoodConstants(
            CANBus CANBus,
            int motorId,
            double gearing,
            double upperLimitRots,
            double lowerLimitRots
    ) {}

    public static final HoodConstants HOOD_CONSTANTS = new HoodConstants(
            CANBus.RIO,
            22,
            (40.0 / 12.0) * (36.0 / 20.0) * (170.0 / 10.0),
            0.1,
            0
    );

    public record ShooterConstants(
            CANBus CANBus,
            int masterId,
            int followerId,
            double gearing
    ) {}

    public static final ShooterConstants SHOOTER_CONSTANTS = new ShooterConstants(
            CANBus.RIO,
            23,
            24,
            2
    );
}
