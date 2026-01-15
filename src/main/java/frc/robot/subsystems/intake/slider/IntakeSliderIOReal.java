package frc.robot.subsystems.intake.slider;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.RefreshAll;

public class IntakeSliderIOReal implements IntakeSliderIO {
    private final HardwareConstants.IntakeSliderConstants constants;

    private final TalonFX sliderMotor;
    private final CANcoder sliderEncoder;

    private final StatusSignal<Angle> sliderPosition;
    private final StatusSignal<AngularVelocity> sliderVelocity;
    private final StatusSignal<Voltage> sliderVoltage;
    private final StatusSignal<Current> sliderTorqueCurrent;
    private final StatusSignal<Temperature> sliderDeviceTemp;
    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<AngularVelocity> encoderVelocity;

    private final PositionVoltage positionVoltage;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    public IntakeSliderIOReal(final HardwareConstants.IntakeSliderConstants constants) {
        this.constants = constants;

        this.sliderMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());
        this.sliderEncoder = new CANcoder(constants.encoderID(), constants.CANBus().toPhoenix6CANBus());

        this.sliderPosition = sliderMotor.getPosition(false);
        this.sliderVelocity = sliderMotor.getVelocity(false);
        this.sliderVoltage = sliderMotor.getMotorVoltage(false);
        this.sliderTorqueCurrent = sliderMotor.getTorqueCurrent(false);
        this.sliderDeviceTemp = sliderMotor.getDeviceTemp(false);
        this.encoderPosition = sliderEncoder.getPosition(false);
        this.encoderVelocity = sliderEncoder.getVelocity(false);

        this.positionVoltage = new PositionVoltage(0);
        this.voltageOut = new VoltageOut(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);

        RefreshAll.add(
                constants.CANBus(),
                sliderPosition,
                sliderVelocity,
                sliderVoltage,
                sliderTorqueCurrent,
                sliderDeviceTemp,
                encoderPosition,
                encoderVelocity
        );
    }

    @Override
    public void config() {
        final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = constants.encoderOffset();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        sliderEncoder.getConfigurator().apply(encoderConfig);

        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(0.28)
                .withKG(0.36)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(50)
                .withKD(0.4);
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = sliderEncoder.getDeviceID();
        motorConfig.Feedback.SensorToMechanismRatio = 1;
        motorConfig.Feedback.RotorToSensorRatio = constants.gearing();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        sliderMotor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                sliderPosition,
                sliderVelocity,
                sliderVoltage,
                sliderTorqueCurrent,
                encoderPosition,
                encoderVelocity
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                sliderDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                4,
                sliderMotor,
                sliderEncoder
        );
    }

    @Override
    public void updateInputs(IntakeArmIOInputs inputs) {
        inputs.sliderPositionRots = sliderPosition.getValueAsDouble();
        inputs.sliderVelocityRotsPerSec = sliderVelocity.getValueAsDouble();
        inputs.sliderVoltage = sliderVoltage.getValueAsDouble();
        inputs.sliderTorqueCurrentAmps = sliderTorqueCurrent.getValueAsDouble();
        inputs.sliderTempCelsius = sliderDeviceTemp.getValueAsDouble();
        inputs.encoderPositionRots = encoderPosition.getValueAsDouble();
        inputs.encoderVelocityRotsPerSec = encoderVelocity.getValueAsDouble();
    }

    @Override
    public void toPosition(double positionRots) {
        sliderMotor.setControl(positionVoltage.withPosition(positionRots));
    }

    @Override
    public void zeroPosition() {
        sliderMotor.setPosition(0);
    }
}
