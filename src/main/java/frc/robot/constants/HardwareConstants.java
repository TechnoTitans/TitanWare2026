package frc.robot.constants;

import java.util.HashMap;
import java.util.Objects;

public class HardwareConstants {
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

    public static final int PowerDistributionHub = 2;

    public record IntakeRollerConstants(
            CANBus CANBus,
            int motorID,
            double gearing
    ) {}

    public static final IntakeRollerConstants INTAKE_ROLLER = new IntakeRollerConstants(
            CANBus.CANIVORE,
            14,
            5.0 / 3
    );

    public record IntakeSlideConstants(
            CANBus CANBus,
            int masterMotorID,
            int followerMotorID,
            double gearing,
            double forwardLimitRots,
            double reverseLimitRots
    ) {}

    public static final IntakeSlideConstants INTAKE_SLIDE = new IntakeSlideConstants(
            CANBus.CANIVORE,
            15,
            16,
            11.111,
            3.5,
            0
    );

    public record SpindexerConstants(
            CANBus CANBus,
            int motorID,
            double gearing
    ) {}

    public static final SpindexerConstants SPINDEXER = new SpindexerConstants(
            CANBus.CANIVORE,
            17,
            2
    );

    public record FeederConstants(
            CANBus CANBus,
            int motorID,
            int CANRangeID,
            double gearing
    ) {}


    public static final FeederConstants FEEDER = new FeederConstants(
            CANBus.CANIVORE,
            18,
            19,
            1.8667
    );

    public record TurretConstants (
            CANBus CANBus,
            int turretMotorID,
            int primaryEncoderID,
            int secondaryEncoderID,
            double motorToTurretGearing,
            int turretTooth,
            int primaryEncoderTooth,
            int secondaryEncoderTooth,
            double primaryEncoderOffset,
            double secondaryEncoderOffset,
            double forwardLimitRots,
            double reverseLimitRots
    ) {}

    public static final TurretConstants TURRET = new TurretConstants(
            CANBus.CANIVORE,
            20,
            21,
            22,
            24,
            80,
            13,
            17,
            -0.320,
            0.340,
            0.75,
            -0.25
    );
    public record HoodConstants(
            CANBus CANBus,
            int motorID,
            double gearing,
            double upperLimitRots,
            double lowerLimitRots
    ){}

    public static final HoodConstants HOOD = new HoodConstants(
            CANBus.RIO,
            23,
            102,
            0.11,
            0
    );

    public record ShooterConstants(
            CANBus CANBus,
            int masterMotorID,
            int followerMotorID,
            double gearing
    ) {}

    public static final ShooterConstants SHOOTER = new ShooterConstants(
            CANBus.RIO,
            24,
            25,
            2
    );

    public record HopperConstants(
            CANBus CANBus,
            int masterMotorID,
            int followerMotorID,
            double gearing
    ) {}

    public static final HopperConstants HOPPER = new HopperConstants(
            CANBus.CANIVORE,
            26,
            27,
            2.5
    );
}