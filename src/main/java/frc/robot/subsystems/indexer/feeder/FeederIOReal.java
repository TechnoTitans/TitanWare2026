package frc.robot.subsystems.indexer.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class FeederIOReal implements FeederIO {
    private final HardwareConstants.FeederConstants constants;
    private final TalonFX motor;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    public FeederIOReal(final HardwareConstants.FeederConstants constants) {
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.motor = new TalonFX(constants.motorId(), p6Bus);

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this.motorPosition = motor.getPosition(false);
        this.motorVelocity = motor.getVelocity(false);
        this.motorVoltage = motor.getMotorVoltage(false);
        this.motorTorqueCurrent = motor.getTorqueCurrent(false);
        this.motorDeviceTemp = motor.getDeviceTemp(false);

        RefreshAll.add(
                bus,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                motorDeviceTemp
        );
    }

    @Override
    public void updateInputs(final FeederIOInputs inputs) {
        inputs.rollerPositionRots = motorPosition.getValueAsDouble();
        inputs.rollerVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.rollerVoltage = motorVoltage.getValueAsDouble();
        inputs.rollerTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = motorDeviceTemp.getValueAsDouble();
    }

    @Override
    public void config() {
        final TalonFXConfiguration feederConfiguration = new TalonFXConfiguration();
        feederConfiguration.Slot0 = new Slot0Configs()
                .withKS(5.27)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(0.185)
                .withKP(5.2);
        feederConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        feederConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        feederConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        feederConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        feederConfiguration.Feedback.SensorToMechanismRatio = constants.gearing();
        feederConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(feederConfiguration));

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                motorDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor
        );
    }

    @Override
    public void toFeederVelocity(final double feederVelocityRotsPerSec) {
        motor.setControl(velocityTorqueCurrentFOC.withVelocity(feederVelocityRotsPerSec));
    }

    @Override
    public void toFeederVoltage(final double feederVolts) {
        motor.setControl(voltageOut.withOutput(feederVolts));
    }
}
