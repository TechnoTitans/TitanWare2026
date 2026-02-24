package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.RefreshAll;

public class FeederIOReal implements FeederIO {
    private final HardwareConstants.FeederConstants constants;

    private final TalonFX rollerMotor;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerDeviceTemp;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;

    public FeederIOReal(final HardwareConstants.FeederConstants constants) {
        this.constants = constants;

        this.rollerMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.rollerPosition = rollerMotor.getPosition(false);
        this.rollerVelocity = rollerMotor.getVelocity(false);
        this.rollerVoltage = rollerMotor.getMotorVoltage(false);
        this.rollerTorqueCurrent = rollerMotor.getTorqueCurrent(false);
        this.rollerDeviceTemp = rollerMotor.getDeviceTemp(false);

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

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
        final TalonFXConfiguration rollerConfiguration = new TalonFXConfiguration();
        rollerConfiguration.Slot0 = new Slot0Configs()
                .withKS(6.72)
                .withKV(0.08)
                .withKP(6)
                .withKD(0.09);
        rollerConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        rollerConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        rollerConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        rollerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimit = 70;
        rollerConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        rollerConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rollerConfiguration.Feedback.SensorToMechanismRatio = constants.rollerGearing();
        rollerMotor.getConfigurator().apply(rollerConfiguration);

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
    public void updateInputs(final FeederIOInputs inputs) {
        inputs.rollerPositionRots = rollerPosition.getValueAsDouble();
        inputs.rollerVelocityRotsPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toRollerVelocity(final double rollerVelocityRotsPerSec) {
        rollerMotor.setControl(velocityTorqueCurrentFOC.withVelocity(rollerVelocityRotsPerSec));
    }
}
