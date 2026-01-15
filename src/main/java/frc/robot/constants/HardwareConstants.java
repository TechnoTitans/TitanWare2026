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

    public record IntakeSliderConstants(
            CANBus CANBus,
            int motorID,
            int encoderID,
            double encoderOffset,
            double gearing,
            double gearPitchCircumference,
            double upperLimitRots,
            double lowerLimitRots
    ) {}

    public static final IntakeSliderConstants INTAKE_SLIDER = new IntakeSliderConstants(
            CANBus.RIO,
            14,
            15,
            0,
            20,
            0.5,
            2,
            0
    );

    public record IntakeConstants(
            CANBus CANBus,
            int motorID,
            double gearing
    ){}

    public static final IntakeConstants INTAKE = new IntakeConstants(
            CANBus.RIO,
            16,
            10
    );

    public record HopperConstants(
            CANBus CANBus,
            int motorID,
            double rollerGearing
    ) {}


    public static HopperConstants HOPPER = new HopperConstants(
            CANBus.RIO,
            17,
            30
    );

    public record TurretConstants (
            CANBus CANBus,
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

    public static TurretConstants TURRET = new TurretConstants(
            CANBus.CANIVORE,
            18,
            19,
            20,
            45,
            13,
            17,
            0,
            0,
            70,
            0.5,
            -0.5
    );

    public record HoodConstants(
            CANBus CANBus,
            int motorID,
            double hoodGearing,
            double hoodUpperLimitRots,
            double hoodLowerLimitRots
    ) {}

    //TODO: Change numbers
    public static HoodConstants HOOD = new HoodConstants(
            CANBus.RIO,
            21,
            50,
            0.25,
            0
    );

    public record ShooterConstants(
            CANBus CANBus,
            int motorID,
            double gearing
    ) {}

    //TODO: Change numbers
    public static ShooterConstants SHOOTER = new ShooterConstants(
            CANBus.RIO,
            22,
            10
    );
}
