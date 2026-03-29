package frc.robot.subsystems.indexer.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class FeederIOReal implements FeederIO {
    private final HardwareConstants.FeederConstants constants;

    private final TalonFX motor;
    private final CANrange CANRange;

    private final StatusSignal<Angle> wheelPosition;
    private final StatusSignal<AngularVelocity> wheelVelocity;
    private final StatusSignal<Voltage> wheelVoltage;
    private final StatusSignal<Current> wheelTorqueCurrent;
    private final StatusSignal<Temperature> wheelDeviceTemp;

    private final StatusSignal<Boolean> CANRangeDetected;

    private final TorqueCurrentFOC torqueCurrentFOC;

    public FeederIOReal(final HardwareConstants.FeederConstants constants) {
        this.constants = constants;

        this.motor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());
        this.CANRange = new CANrange(constants.CANRangeID(), constants.CANBus().toPhoenix6CANBus());

        this.wheelPosition = motor.getPosition(false);
        this.wheelVelocity = motor.getVelocity(false);
        this.wheelVoltage = motor.getMotorVoltage(false);
        this.wheelTorqueCurrent = motor.getTorqueCurrent(false);
        this.wheelDeviceTemp = motor.getDeviceTemp(false);

        this.CANRangeDetected = CANRange.getIsDetected(false);

        this.torqueCurrentFOC = new TorqueCurrentFOC(0);

        RefreshAll.add(
                constants.CANBus(),
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                wheelDeviceTemp,
                CANRangeDetected
        );
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(14)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKV(0)
                .withKP(10)
                .withKD(0);
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        motorConfig.CurrentLimits.StatorCurrentLimit = 80;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 80;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 55;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.gearing();
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfig));

        final CANrangeConfiguration CANRangeConfig = new CANrangeConfiguration();
        CANRangeConfig.ProximityParams.ProximityThreshold = 0.4;
        CANRangeConfig.ProximityParams.ProximityHysteresis = 0.01;
        CANRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
        CANRangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        Phoenix6Utils.tryUntilOk(CANRange, () -> CANRange.getConfigurator().apply(CANRangeConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                CANRangeDetected
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                wheelDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor,
                CANRange
        );
    }

    @Override
    public void updateInputs(final FeederIOInputs inputs) {
        inputs.wheelPositionRots = wheelPosition.getValueAsDouble();
        inputs.wheelVelocityRotsPerSec = wheelVelocity.getValueAsDouble();
        inputs.wheelVoltage = wheelVoltage.getValueAsDouble();
        inputs.wheelTorqueCurrentAmps = wheelTorqueCurrent.getValueAsDouble();
        inputs.wheelTempCelsius = wheelDeviceTemp.getValueAsDouble();

        inputs.TOFDetected = CANRangeDetected.getValue();
    }

    @Override
    public void toWheelTorqueCurrent(final double torqueCurrentAmps) {
        motor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }
}
