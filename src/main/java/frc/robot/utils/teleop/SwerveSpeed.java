package frc.robot.utils.teleop;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.Container;
import frc.robot.utils.commands.ext.CommandsExt;

import java.util.function.Supplier;

public class SwerveSpeed {
    public enum Speeds {
        FAST(Units.feetToMeters(15), 2 * Math.PI),
        NORMAL(Units.feetToMeters(12.5), 1.7 * Math.PI),
        SLOW(Units.feetToMeters(5), 1 * Math.PI),
        SHOOTING(Units.feetToMeters(5), 1.2 * Math.PI),
        FERRYING(Units.feetToMeters(6.5), 1.3 * Math.PI);

        private final double translationSpeed;
        private final double rotationSpeed;

        Speeds(final double translationSpeed, final double rotationSpeed) {
            this.translationSpeed = translationSpeed;
            this.rotationSpeed = rotationSpeed;
        }

        public double getTranslationSpeed() {
            return translationSpeed;
        }

        public double getRotationSpeed() {
            return rotationSpeed;
        }
    }

    private static Speeds SwerveSpeed = Speeds.NORMAL;

    private SwerveSpeed() {}

    public static Speeds getSwerveSpeed() {
        return SwerveSpeed;
    }

    public static Command toSwerveSpeed(final Supplier<Speeds> speedsSupplier) {
        final Container<Speeds> speedsContainer = Container.of(SwerveSpeed);
        return CommandsExt.instantRunEnd(
                () -> speedsContainer.set(SwerveSpeed),
                () -> SwerveSpeed = speedsSupplier.get(),
                () -> SwerveSpeed = speedsContainer.get()
        );
    }

    public static Command toSwerveSpeed(final Speeds speeds) {
        final Container<Speeds> speedsContainer = Container.of(SwerveSpeed);
        return Commands.startEnd(
                () -> {
                    speedsContainer.set(SwerveSpeed);
                    SwerveSpeed = speeds;
                },
                () -> SwerveSpeed = speedsContainer.get()
        );
    }

    public static Command toSwerveSpeed(final Speeds speeds, final Speeds resetToSpeeds) {
        return Commands.startEnd(
                () -> SwerveSpeed = speeds,
                () -> SwerveSpeed = resetToSpeeds
        );
    }
}
