package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class ClimbIOReal implements ClimbIO {
    private final HardwareConstants.ClimbConstants constants;

    private final TalonFX climbMotor;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    public ClimbIOReal(final HardwareConstants.ClimbConstants constants) {
        this.constants = constants;

        this.climbMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);

        this.motorPosition = climbMotor.getPosition(false);
        this.motorVelocity = climbMotor.getVelocity(false);
        this.motorVoltage = climbMotor.getMotorVoltage(false);
        this.motorTorqueCurrent = climbMotor.getTorqueCurrent(false);
        this.motorDeviceTemp = climbMotor.getDeviceTemp(false);

        RefreshAll.add(
                constants.CANBus(),
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                motorDeviceTemp
        );
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.2)
                .withKP(15)
                .withKD(0);
        motorConfiguration.Slot1 = new Slot1Configs()
                .withKS(0.2)
                .withKP(50)
                .withKD(1);
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = 70;
        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.climbGearing();
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        climbMotor.getConfigurator().apply(motorConfiguration);

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
                climbMotor
        );
    }

    @Override
    public void updateInputs(final ClimbIOInputs inputs) {
        inputs.motorPositionRots = motorPosition.getValueAsDouble();
        inputs.motorVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.motorVoltage = motorVoltage.getValueAsDouble();
        inputs.motorTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.motorTempCelsius = motorDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toPosition(final double positionRots) {
        climbMotor.setControl(motionMagicExpoVoltage.withPosition(positionRots));
    }

    @Override
    public void setPosition(final double positionRots) {
        Phoenix6Utils.reportIfNotOk(climbMotor, climbMotor.setPosition(positionRots));
    }
}

