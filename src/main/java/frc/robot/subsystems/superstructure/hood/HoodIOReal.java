package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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

public class HoodIOReal implements HoodIO {
    private final HardwareConstants.HoodConstants constants;

    private final TalonFX hoodMotor;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodVoltage;
    private final StatusSignal<Current> hoodTorqueCurrent;
    private final StatusSignal<Temperature> hoodDeviceTemp;

    private final PositionVoltage positionVoltage;

    public HoodIOReal(final HardwareConstants.HoodConstants constants) {
        this.constants = constants;

        this.hoodMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.hoodPosition = hoodMotor.getPosition(false);
        this.hoodVelocity = hoodMotor.getVelocity(false);
        this.hoodVoltage = hoodMotor.getMotorVoltage(false);
        this.hoodTorqueCurrent = hoodMotor.getTorqueCurrent(false);
        this.hoodDeviceTemp = hoodMotor.getDeviceTemp(false);

        this.positionVoltage = new PositionVoltage(0);

        RefreshAll.add(
                constants.CANBus(),
                hoodPosition,
                hoodVelocity,
                hoodVoltage,
                hoodTorqueCurrent,
                hoodDeviceTemp
        );
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(0.3)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKP(300)
                .withKD(0.1);
        motorConfig.CurrentLimits.StatorCurrentLimit = 40;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 45;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.gearing();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        Phoenix6Utils.tryUntilOk(hoodMotor, () -> hoodMotor.getConfigurator().apply(motorConfig));

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
        hoodMotor.setControl(positionVoltage.withPosition(positionRots));
    }

    @Override
    public void zeroMotor() {
        Phoenix6Utils.tryUntilOk(hoodMotor, () -> hoodMotor.setPosition(0));
    }
}
