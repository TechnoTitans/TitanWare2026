package frc.robot.subsystems.feeder;

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
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.RefreshAll;

public class FeederIOReal implements FeederIO {
    private final HardwareConstants.FeederConstants constants;

    private final TalonFX wheelMotor;

    private final StatusSignal<Angle> wheelPosition;
    private final StatusSignal<AngularVelocity> wheelVelocity;
    private final StatusSignal<Voltage> wheelVoltage;
    private final StatusSignal<Current> wheelTorqueCurrent;
    private final StatusSignal<Temperature> wheelDeviceTemp;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;

    public FeederIOReal(final HardwareConstants.FeederConstants constants) {
        this.constants = constants;

        this.wheelMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.wheelPosition = wheelMotor.getPosition(false);
        this.wheelVelocity = wheelMotor.getVelocity(false);
        this.wheelVoltage = wheelMotor.getMotorVoltage(false);
        this.wheelTorqueCurrent = wheelMotor.getTorqueCurrent(false);
        this.wheelDeviceTemp = wheelMotor.getDeviceTemp(false);

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

        RefreshAll.add(
                constants.CANBus(),
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                wheelDeviceTemp
        );
    }

    @Override
    public void config() {
        final TalonFXConfiguration wheelConfiguration = new TalonFXConfiguration();
        wheelConfiguration.Slot0 = new Slot0Configs()
                .withKS(6.72)
                .withKV(0.08)
                .withKP(6)
                .withKD(0.09);
        wheelConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        wheelConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        wheelConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        wheelConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        wheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        wheelConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        wheelConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        wheelConfiguration.Feedback.SensorToMechanismRatio = constants.wheelGearing();
        wheelMotor.getConfigurator().apply(wheelConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                wheelDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                wheelMotor
        );
    }

    @Override
    public void updateInputs(final FeederIOInputs inputs) {
        inputs.wheelPositionRots = wheelPosition.getValueAsDouble();
        inputs.wheelVelocityRotsPerSec = wheelVelocity.getValueAsDouble();
        inputs.wheelVoltage = wheelVoltage.getValueAsDouble();
        inputs.wheelTorqueCurrentAmps = wheelTorqueCurrent.getValueAsDouble();
        inputs.wheelTempCelsius = wheelDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toWheelVelocity(final double velocityRotsPerSec) {
        wheelMotor.setControl(velocityTorqueCurrentFOC.withVelocity(velocityRotsPerSec));
    }
}
