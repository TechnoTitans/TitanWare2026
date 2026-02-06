package frc.robot.constants;

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

    public static IntakeRollerConstants INTAKE_ROLLER = new IntakeRollerConstants(
            CANBus.RIO,
            14,
            10
    );

    public record IntakeSlideConstants(
            CANBus CANBus,
            int masterMotorID,
            int followerMotorID,
            int encoderID,
            double encoderOffset,
            double slideGearing,
            double gearPitchCircumferenceMeters,
            double upperLimitRots,
            double lowerLimitRots
    ) {}

    public static IntakeSlideConstants INTAKE_SLIDE = new IntakeSlideConstants(
            CANBus.RIO,
            15,
            16,
            17,
            0,
            20,
            0.1,
            10,
            0
    );

    public record SpindexerConstants(
            CANBus CANBus,
            int motorID,
            double rollerGearing
    ) {}

    public static SpindexerConstants SPINDEXER = new SpindexerConstants(
            CANBus.RIO,
            18,
            10
    );

    public record FeederConstants(
            CANBus CANBus,
            int motorID,
            double rollerGearing
    ) {}


    public static FeederConstants FEEDER = new FeederConstants(
            CANBus.RIO,
            19,
            30
    );

    public record TurretConstants (
            CANBus CANBus,
            int turretMotorID,
            int leftEncoderID,
            int rightEncoderID,
            double turretToMechanismGearing,
            int turretTooth,
            double leftEncoderGearing,
            double rightEncoderGearing,
            double leftEncoderOffset,
            double rightEncoderOffset,
            double upperLimitRots,
            double lowerLimitRots
    ) {}

    public static TurretConstants TURRET = new TurretConstants(
            CANBus.CANIVORE,
            20,
            21,
            22,
            24,
            80,
            13.0,
            17.0,
            0,
            0,
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
            23,
            50,
            0.25,
            0
    );

    public record ShooterConstants(
            CANBus CANBus,
            int masterMotorID,
            int followerMotorID,
            double gearing
    ) {}

    //TODO: Change numbers
    public static ShooterConstants SHOOTER = new ShooterConstants(
            CANBus.RIO,
            24,
            25,
            2
    );

    public record ClimbConstants(
            CANBus CANBus,
            int motorID,
            double climbGearing,
            double lowerLimitRots,
            double upperLimitRots,
            double spoolDiameterMeters
    ){}

    public static final ClimbConstants CLIMB = new ClimbConstants(
            CANBus.RIO,
            25,
            26,
            0.0,
            5.0,
            0.1
    );
}