package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.RefreshAll;

public class SpindexerIOReal implements SpindexerIO {
    private final HardwareConstants.SpindexerConstants constants;

    private final TalonFX wheelMotor;

    private final StatusSignal<Angle> wheelPositionRots;
    private final StatusSignal<AngularVelocity> wheelVelocity;
    private final StatusSignal<Voltage> wheelVoltage;
    private final StatusSignal<Current> wheelTorqueCurrent;
    private final StatusSignal<Temperature> wheelTemperature;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;

    public SpindexerIOReal(final HardwareConstants.SpindexerConstants constants) {
        this.constants = constants;

        this.wheelMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.wheelPositionRots = wheelMotor.getPosition(false);
        this.wheelVelocity = wheelMotor.getVelocity(false);
        this.wheelVoltage = wheelMotor.getMotorVoltage(false);
        this.wheelTorqueCurrent = wheelMotor.getTorqueCurrent(false);
        this.wheelTemperature = wheelMotor.getDeviceTemp(false);

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

        RefreshAll.add(
                constants.CANBus(),
                wheelPositionRots,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                wheelTemperature
        );
    }

    @Override
    public void config() {
        final TalonFXConfiguration wheelConfiguration = new TalonFXConfiguration();
        wheelConfiguration.Slot0 = new Slot0Configs()
                .withKS(4.0212)
                .withKV(0.40767)
                .withKA(0.22711)
                .withKP(30);
        wheelConfiguration.CurrentLimits.StatorCurrentLimit = 40;
        wheelConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        wheelConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        wheelConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        wheelConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;
        wheelConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        wheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wheelConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        wheelConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        wheelConfiguration.Feedback.SensorToMechanismRatio = constants.wheelGearing();
        wheelMotor.getConfigurator().apply(wheelConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                wheelPositionRots,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                wheelTemperature
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                wheelMotor
        );
    }

    @Override
    public void updateInputs(final SpindexerIOInputs inputs) {
        inputs.wheelPositionRots = wheelPositionRots.getValueAsDouble();
        inputs.wheelVelocityRotsPerSec = wheelVelocity.getValueAsDouble();
        inputs.wheelVoltage = wheelVoltage.getValueAsDouble();
        inputs.wheelTorqueCurrentAmps = wheelTorqueCurrent.getValueAsDouble();
        inputs.wheelTemperatureCelsius = wheelTemperature.getValueAsDouble();
    }

    @Override
    public void toWheelVelocity(final double wheelVelocityRotsPerSec) {
        wheelMotor.setControl(velocityTorqueCurrentFOC.withVelocity(wheelVelocityRotsPerSec));
    }
}
