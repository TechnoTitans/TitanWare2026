package frc.robot.subsystems.intake.roller;

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

public class IntakeRollerIOReal implements IntakeRollerIO {
    private final HardwareConstants.IntakeRollerConstants constants;

    private final TalonFX rollerMotor;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerDeviceTemp;

    private final VoltageOut voltageOut;

    public IntakeRollerIOReal(final HardwareConstants.IntakeRollerConstants constants) {
        this.constants = constants;

        this.rollerMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.rollerPosition = rollerMotor.getPosition(false);
        this.rollerVelocity = rollerMotor.getVelocity(false);
        this.rollerVoltage = rollerMotor.getMotorVoltage(false);
        this.rollerTorqueCurrent = rollerMotor.getTorqueCurrent(false);
        this.rollerDeviceTemp = rollerMotor.getDeviceTemp(false);

        this.voltageOut = new VoltageOut(0);

        RefreshAll.add(
                constants.CANBus(),
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                rollerDeviceTemp
        );
    }

    @Override
    public void config() {
        final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLowerTime = 2.5;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = constants.rollerGearing();
        Phoenix6Utils.tryUntilOk(rollerMotor, () -> rollerMotor.getConfigurator().apply(talonFXConfiguration));

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
                rollerMotor
        );
    }

    @Override
    public void updateInputs(final IntakeRollerIOInputs inputs) {
        inputs.rollerPositionRots = rollerPosition.getValueAsDouble();
        inputs.rollerVelocityRotsPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toRollerVoltage(final double volts) {
        rollerMotor.setControl(voltageOut.withOutput(volts));
    }
}
