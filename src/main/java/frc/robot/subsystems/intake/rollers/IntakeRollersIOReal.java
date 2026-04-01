package frc.robot.subsystems.intake.rollers;

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

public class IntakeRollersIOReal implements IntakeRollersIO {
    private final HardwareConstants.IntakeRollerConstants constants;

    private final TalonFX motor;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerDeviceTemp;

    private final VoltageOut voltageOut;

    public IntakeRollersIOReal(final HardwareConstants.IntakeRollerConstants constants) {
        this.constants = constants;

        this.motor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.rollerPosition = motor.getPosition(false);
        this.rollerVelocity = motor.getVelocity(false);
        this.rollerVoltage = motor.getMotorVoltage(false);
        this.rollerTorqueCurrent = motor.getTorqueCurrent(false);
        this.rollerDeviceTemp = motor.getDeviceTemp(false);

        this.voltageOut = new VoltageOut(0);

        RefreshAll.add(
                constants.CANBus(),
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                rollerDeviceTemp
        );

        config();
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 2.5;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.gearing();
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                rollerDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor
        );
    }

    @Override
    public void updateInputs(final IntakeRollerIOInputs inputs) {
        inputs.rollersPositionRots = rollerPosition.getValueAsDouble();
        inputs.rollersVelocityRotsPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollersVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollersTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollersTempCelsius = rollerDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toRollersVoltage(final double volts) {
        motor.setControl(voltageOut.withOutput(volts));
    }
}
