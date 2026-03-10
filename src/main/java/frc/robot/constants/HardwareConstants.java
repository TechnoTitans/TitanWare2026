package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.HashMap;
import java.util.Objects;

public class HardwareConstants {
    public enum CANBus {
        RIO("rio"),
        CANIVORE("CANIVORE");

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

    public record IntakeRollerConstants(
            CANBus CANBus,
            int motorID,
            double rollerGearing
    ) {}

    public static final IntakeRollerConstants INTAKE_ROLLER = new IntakeRollerConstants(
            CANBus.RIO,
            14,
            5.0 / 3
    );

    public record IntakeSlideConstants(
            CANBus CANBus,
            int masterMotorID,
            int followerMotorID,
            double slideGearing,
            double upperLimitRots,
            double lowerLimitRots
    ) {}

    public static final IntakeSlideConstants INTAKE_SLIDE = new IntakeSlideConstants(
            CANBus.CANIVORE,
            15,
            16,
            10,
            3.8,
            0
    );

    public record SpindexerConstants(
            CANBus CANBus,
            int motorID,
            double wheelGearing
    ) {}

    public static final SpindexerConstants SPINDEXER = new SpindexerConstants(
            CANBus.CANIVORE,
            17,
            2
    );

    public record FeederConstants(
            CANBus CANBus,
            int motorID,
            double rollerGearing
    ) {}


    public static final FeederConstants FEEDER = new FeederConstants(
            CANBus.CANIVORE,
            18,
            3
    );

    public record TurretConstants (
            CANBus CANBus,
            int turretMotorID,
            int smallEncoderID,
            int largeEncoderID,
            double motorToTurretGearing,
            int turretTooth,
            int smallEncoderTooth,
            int largeEncoderTooth,
            double smallEncoderOffset,
            double largeEncoderOffset,
            double forwardLimitRots,
            double reverseLimitRots,
            Transform2d offsetFromCenter
    ) {}

    public static final TurretConstants TURRET = new TurretConstants(
            CANBus.CANIVORE,
            19,
            20,
            21,
            24,
            80,
            13,
            17,
            0, //-0.451,
            0, //-0.061,
            0.5,
            -0.5,
            new Transform2d(-0.127, 0, Rotation2d.kZero)
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
            22,
            102,
            0.1103,
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
            23,
            24,
            2
    );
}