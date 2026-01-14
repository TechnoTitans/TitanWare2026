package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.RefreshAll;

public class HoodIOReal implements HoodIO {
    private final HardwareConstants.HoodConstants constants;

    private final TalonFX hoodMotor;
    final TalonFXConfiguration motorConfig;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodVoltage;
    private final StatusSignal<Current> hoodTorqueCurrent;
    private final StatusSignal<Temperature> hoodDeviceTemp;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final PositionVoltage positionVoltage;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    public HoodIOReal(final HardwareConstants.HoodConstants constants) {
        this.constants = constants;

        this.hoodMotor = new TalonFX(constants.hoodMotorID(), constants.CANBus());
        this.motorConfig = new TalonFXConfiguration();

        this.hoodPosition = hoodMotor.getPosition(false);
        this.hoodVelocity = hoodMotor.getVelocity(false);
        this.hoodVoltage = hoodMotor.getMotorVoltage(false);
        this.hoodTorqueCurrent = hoodMotor.getTorqueCurrent(false);
        this.hoodDeviceTemp = hoodMotor.getDeviceTemp(false);

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.positionVoltage = new PositionVoltage(0);
        this.voltageOut = new VoltageOut(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);

        RefreshAll.add(
                RefreshAll.getBus(constants.CANBus()),
                hoodPosition,
                hoodVelocity,
                hoodVoltage,
                hoodTorqueCurrent,
                hoodDeviceTemp
        );
    }

    @Override
    public void config() {
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(0.016887)
                .withKG(0.25249)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKV(10.263)
                .withKA(2.2613)
                .withKP(76.008)
                .withKD(40);
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorConfig.MotionMagic.MotionMagicExpo_kV = 9.263;
        motorConfig.MotionMagic.MotionMagicExpo_kA = 2.1;
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        motorConfig.CurrentLimits.StatorCurrentLimit = 50;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.hoodGearing();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.hoodUpperLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.hoodLowerLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodMotor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                hoodPosition,
                hoodVelocity,
                hoodVoltage,
                hoodTorqueCurrent
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                hoodDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                4,
                hoodMotor
        );
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodPositionRots = hoodPosition.getValueAsDouble();
        inputs.hoodVelocityRotsPerSec = hoodVelocity.getValueAsDouble();
        inputs.hoodVoltage = hoodVoltage.getValueAsDouble();
        inputs.hoodTorqueCurrentAmps = hoodTorqueCurrent.getValueAsDouble();
        inputs.hoodTempCelsius = hoodDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toHoodPosition(final double positionRots) {
        hoodMotor.setControl(motionMagicExpoVoltage.withPosition(positionRots));
    }

    @Override
    public void toHoodContinuousPosition(final double positionRots) {
        hoodMotor.setControl(positionVoltage.withPosition(positionRots));
    }

    @Override
    public void toHoodVoltage(final double volts) {
        hoodMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toHoodTorqueCurrent(final double torqueCurrent) {
        hoodMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrent));
    }

    @Override
    public void home() {
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 1;
        hoodMotor.setControl(voltageOut.withOutput(-0.1));
    }

    @Override
    public void zeroMotor() {
        hoodMotor.setPosition(0);
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.hoodUpperLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        toHoodPosition(0);
    }
}
