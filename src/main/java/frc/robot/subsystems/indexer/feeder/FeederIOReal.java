package frc.robot.subsystems.indexer.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
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
    private final CANrange canRange;

    private final StatusSignal<Angle> wheelPosition;
    private final StatusSignal<AngularVelocity> wheelVelocity;
    private final StatusSignal<Voltage> wheelVoltage;
    private final StatusSignal<Current> wheelTorqueCurrent;
    private final StatusSignal<Temperature> wheelDeviceTemp;

    private final StatusSignal<Distance> canRangeDistance;
    private final StatusSignal<Boolean> canRangeDetected;

    private final TorqueCurrentFOC torqueCurrentFOC;

    public FeederIOReal(final HardwareConstants.FeederConstants constants) {
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.motor = new TalonFX(constants.motorId(), p6Bus);
        this.canRange = new CANrange(constants.CANRangeId(), p6Bus);

        this.wheelPosition = motor.getPosition(false);
        this.wheelVelocity = motor.getVelocity(false);
        this.wheelVoltage = motor.getMotorVoltage(false);
        this.wheelTorqueCurrent = motor.getTorqueCurrent(false);
        this.wheelDeviceTemp = motor.getDeviceTemp(false);

        this.canRangeDistance = canRange.getDistance(false);
        this.canRangeDetected = canRange.getIsDetected(false);

        this.torqueCurrentFOC = new TorqueCurrentFOC(0);

        RefreshAll.add(
                bus,
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                wheelDeviceTemp,
                canRangeDistance,
                canRangeDetected
        );

        config();
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
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.gearing();
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfig));

        final CANrangeConfiguration canRangeConfiguration = new CANrangeConfiguration();
        canRangeConfiguration.ProximityParams.ProximityThreshold = 0.05;
        canRangeConfiguration.ProximityParams.ProximityHysteresis = 0.02;
        canRangeConfiguration.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
        canRangeConfiguration.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        Phoenix6Utils.tryUntilOk(canRange, () -> canRange.getConfigurator().apply(canRangeConfiguration));

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                canRangeDistance,
                canRangeDetected
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                wheelDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor,
                canRange
        );
    }

    @Override
    public void updateInputs(final FeederIOInputs inputs) {
        inputs.wheelPositionRots = wheelPosition.getValueAsDouble();
        inputs.wheelVelocityRotsPerSec = wheelVelocity.getValueAsDouble();
        inputs.wheelVoltage = wheelVoltage.getValueAsDouble();
        inputs.wheelTorqueCurrentAmps = wheelTorqueCurrent.getValueAsDouble();
        inputs.wheelTempCelsius = wheelDeviceTemp.getValueAsDouble();

        inputs.tofDistance = canRangeDistance.getValueAsDouble();
        inputs.tofDetected = canRangeDetected.getValue();
    }

    @Override
    public void toWheelTorqueCurrent(final double torqueCurrentAmps) {
        motor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }
}