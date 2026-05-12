package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

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

    public static final int PowerDistributionHub = 1;

    public record IntakeRollerConstants(
            CANBus CANBus,
            int masterMotorId,
            int followerMotorId,
            double gearing
    ) {}

    public static final IntakeRollerConstants INTAKE_ROLLER = new IntakeRollerConstants(
            CANBus.RIO,
            14,
            15,
            5.0 / 3
    );

    public record IntakeSlideConstants(
            CANBus CANBus,
            int masterMotorId,
            int followerMotorId,
            double gearing,
            double forwardLimitRots,
            double reverseLimitRots
    ) {}

    public static final IntakeSlideConstants INTAKE_SLIDE = new IntakeSlideConstants(
            CANBus.CANIVORE,
            16,
            17,
            11.111,
            3.5,
            0
    );

    public record SpindexerConstants(
            CANBus CANBus,
            int motorId,
            double gearing
    ) {}

    public static final SpindexerConstants SPINDEXER = new SpindexerConstants(
            CANBus.CANIVORE,
            18,
            10
    );

    public record FeederConstants(
            CANBus CANBus,
            int motorId,
            int CANRangeId,
            double gearing
    ) {}


    public static final FeederConstants FEEDER = new FeederConstants(
            CANBus.CANIVORE,
            19,
            20,
            1.5
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
                    ((double) primaryCANcoderGearTeeth) / drivenTurretGearTeeth
                            * ((double) secondaryCANcoderGearTeeth) / primaryCANcoderGearTeeth,
                    offsetFromCenter
            );
        }
    }

    public static final TurretConstants TURRET = new TurretConstants(
            CANBus.CANIVORE,
            21,
            22,
            23,
            -0.290283,
            -0.637695,
            0.75,
            -0.25,
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
    ){}

    public static final HoodConstants HOOD = new HoodConstants(
            CANBus.RIO,
            24,
            102,
            0.11,
            0
    );

    public record ShooterConstants(
            CANBus CANBus,
            int masterMotorId,
            int followerMotorId,
            double gearing
    ) {}

    public static final ShooterConstants SHOOTER = new ShooterConstants(
            CANBus.RIO,
            25,
            26,
            2
    );
}