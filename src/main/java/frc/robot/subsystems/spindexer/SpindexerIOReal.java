package frc.robot.subsystems.spindexer;

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

    private final TalonFX wheelMotor;

    private final StatusSignal<Angle> wheelPositionRots;
    private final StatusSignal<AngularVelocity> wheelVelocity;
    private final StatusSignal<Voltage> wheelVoltage;
    private final StatusSignal<Current> wheelTorqueCurrent;
    private final StatusSignal<Temperature> wheelTemp;

    private final VoltageOut voltageOut;

    public SpindexerIOReal(final HardwareConstants.SpindexerConstants constants) {
        this.constants = constants;

        this.wheelMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.wheelPositionRots = wheelMotor.getPosition(false);
        this.wheelVelocity = wheelMotor.getVelocity(false);
        this.wheelVoltage = wheelMotor.getMotorVoltage(false);
        this.wheelTorqueCurrent = wheelMotor.getTorqueCurrent(false);
        this.wheelTemp = wheelMotor.getDeviceTemp(false);

        this.voltageOut = new VoltageOut(0);

        RefreshAll.add(
                constants.CANBus(),
                wheelPositionRots,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                wheelTemp
        );
    }

    @Override
    public void config() {
        final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 75;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 65;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1.5;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = constants.wheelGearing();
        Phoenix6Utils.tryUntilOk(wheelMotor, () -> wheelMotor.getConfigurator().apply(talonFXConfiguration));

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                wheelPositionRots,
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
                wheelMotor
        );
    }

    @Override
    public void updateInputs(final SpindexerIOInputs inputs) {
        inputs.wheelPositionRots = wheelPositionRots.getValueAsDouble();
        inputs.wheelVelocityRotsPerSec = wheelVelocity.getValueAsDouble();
        inputs.wheelVoltage = wheelVoltage.getValueAsDouble();
        inputs.wheelTorqueCurrentAmps = wheelTorqueCurrent.getValueAsDouble();
        inputs.wheelTempCelsius = wheelTemp.getValueAsDouble();
    }

    @Override
    public void toWheelVoltage(final double volts) {
        wheelMotor.setControl(voltageOut.withOutput(volts));
    }
}
