package frc.robot.subsystems.climb;

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

public class ClimbIOReal {
    private final HardwareConstants.ClimbConstants constants;

    private final TalonFX motor;
    private final TalonFXConfiguration motorConfig;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorTemperature;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final PositionVoltage positionVoltage;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    public ClimbIOReal(final HardwareConstants.ClimbConstants constants) {
        this.constants = constants;

        this.motor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());
        this.motorConfig = new TalonFXConfiguration();

        this.motorPosition = motor.getPosition(false);
        this.motorVelocity = motor.getVelocity(false);
        this.motorVoltage = motor.getMotorVoltage(false);
        this.motorTorqueCurrent = motor.getTorqueCurrent(false);
        this.motorTemperature = motor.getDeviceTemp(false);

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.positionVoltage = new PositionVoltage(0);
        this.voltageOut = new VoltageOut(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        
        RefreshAll.add(
                constants.CANBus(),
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                motorTemperature
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
        motorConfig.Feedback.SensorToMechanismRatio = constants.climbGearing();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.climbUpperLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.climbLowerLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                motorTemperature
        );
        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor
        );
    }

    @Override
    public void updateInputs(motorIO.climbIOInputs inputs) {
        inputs.motorPositionRots = motorPosition.getValueAsDouble();
        inputs.motorVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.motorVoltage = motorVoltage.getValueAsDouble();
        inputs.motorTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.motorTempCelsius = motorTemperature.getValueAsDouble();
    }

    @Override
    public void toMotorPosition(final double positionRots) {
        motor.setControl(motionMagicExpoVoltage.withPosition(positionRots));
    }

    @Override
    public void toMotorVoltage(final double volts) {
        motor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toMotorTorqueCurrent(final double torqueCurrent) {
        motor.setControl(torqueCurrentFOC.withOutput(torqueCurrent));
    }

    @Override
    public void home() {
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 1;
        motor.setControl(voltageOut.withOutput(-0.1));
    }

    @Override
    public void zeroMotor() {
        motor.setPosition(0);
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.climbUpperLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        toMotorPosition(0);
    }
}
