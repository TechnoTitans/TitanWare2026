package frc.robot.subsystems.indexer.spindexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class SpindexerIOReal implements SpindexerIO {
    private final HardwareConstants.SpindexerConstants constants;

    private final TalonFX motor;

    private final StatusSignal<Angle> wheelPosition;
    private final StatusSignal<AngularVelocity> wheelVelocity;
    private final StatusSignal<Voltage> wheelVoltage;
    private final StatusSignal<Current> wheelTorqueCurrent;
    private final StatusSignal<Temperature> wheelTemp;

    private final VoltageOut voltageOut;

    public SpindexerIOReal(final HardwareConstants.SpindexerConstants constants) {
        this.constants = constants;

        final HardwareConstants.CANBus canBus = constants.CANBus();
        this.motor = new TalonFX(constants.motorId(), canBus.toPhoenix6CANBus());

        this.wheelPosition = motor.getPosition(false);
        this.wheelVelocity = motor.getVelocity(false);
        this.wheelVoltage = motor.getMotorVoltage(false);
        this.wheelTorqueCurrent = motor.getTorqueCurrent(false);
        this.wheelTemp = motor.getDeviceTemp(false);

        this.voltageOut = new VoltageOut(0);

        RefreshAll.add(
                canBus,
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                wheelTemp
        );

        config();
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.StatorCurrentLimit = 80;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 75;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 65;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.5;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.gearing();
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                wheelTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor
        );
    }

    @Override
    public void updateInputs(final SpindexerIOInputs inputs) {
        inputs.wheelPositionRots = wheelPosition.getValueAsDouble();
        inputs.wheelVelocityRotsPerSec = wheelVelocity.getValueAsDouble();
        inputs.wheelVoltage = wheelVoltage.getValueAsDouble();
        inputs.wheelTorqueCurrentAmps = wheelTorqueCurrent.getValueAsDouble();
        inputs.wheelTempCelsius = wheelTemp.getValueAsDouble();
    }

    @Override
    public void toWheelVoltage(final double volts) {
        motor.setControl(voltageOut.withOutput(volts));
    }
}
