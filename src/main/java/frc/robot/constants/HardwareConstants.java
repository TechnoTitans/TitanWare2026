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
                    BusNameToCANBus.get(bus.getName()),
                    () -> String.format("Could not get CANBus: %s", bus.getName())
            );
        }

        public com.ctre.phoenix6.CANBus toPhoenix6CANBus() {
            return new com.ctre.phoenix6.CANBus(name);
        }
    }

    public record HoodConstants(
            CANBus CANBus,
            int hoodMotorID,
            int hoodCANCoderID,
            double hoodCANCoderOffset,
            double hoodGearing,
            double hoodUpperLimitRots,
            double hoodLowerLimitRots
    ) {}

    // TODO: Change numbers
    public static final HoodConstants HOOD = new HoodConstants(
            CANBus.RIO,
            19,
            50,
            -6.7,
            0.25,
            0.0,
            0.0
    );

    public record ShooterConstants(
            CANBus canBus,
            int motorID,
            double gearing
    ) {}

    // TODO: Change numbers
    public static final ShooterConstants SHOOTER = new ShooterConstants(
            CANBus.RIO,
            20,
            10
    );

    public record TurretConstants(
            CANBus canBus,
            int turretMotorID,
            int leftEncoderID,
            int rightEncoderID,
            double turretGearing,
            double leftEncoderGearing,
            double rightEncoderGearing,
            double leftEncoderOffset,
            double rightEncoderOffset,
            int countableRotations,
            double upperLimitRots,
            double lowerLimitRots
    ) {}

    public static final TurretConstants TURRET = new TurretConstants(
            CANBus.CANIVORE,
            16,
            17,
            18,
            45,
            13,
            17,
            0,
            0,
            70,
            0.5,
            -0.5
    );

    public record IntakeArmConstants(
            CANBus canBus,
            int intakePivotMotorID,
            int intakePivotCANCoderId,
            double intakePivotCANCoderOffset,
            double pivotGearing,
            double pivotLowerLimitRots,
            double pivotUpperLimitRots
    ) {}

    public static final IntakeArmConstants INTAKE_ARM = new IntakeArmConstants(
            CANBus.RIO,
            19,
            20,
            0.62939453125,
            60,
            -0.35,
            0.0
    );

    public record IntakeConstants(
            CANBus canBus,
            int rollerMotorID,
            int coralTOFID,
            double rollerGearing
    ) {}

    public static final IntakeConstants INTAKE = new IntakeConstants(
            CANBus.RIO,
            21,
            23,
            10.0
    );

    public record ClimbConstants(
            CANBus CANBus,
            int motorID,
            double climbGearing,
            double climbLowerLimitRots,
            double climbUpperLimitRots
    ) {}

    // TODO: Change numbers
    public static final ClimbConstants CLIMB = new ClimbConstants(
            CANBus.RIO,
            22,
            50.0,
            0.0,
            0.25
    );
}
