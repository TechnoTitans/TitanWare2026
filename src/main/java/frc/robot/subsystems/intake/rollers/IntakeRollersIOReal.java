package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
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

public class IntakeRollersIOReal implements IntakeRollersIO {
    private final HardwareConstants.IntakeRollerConstants constants;

    private final TalonFX motor;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerDeviceTemp;

    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;

    public IntakeRollersIOReal(final HardwareConstants.IntakeRollerConstants constants) {
        this.constants = constants;

        final HardwareConstants.CANBus canBus = constants.CANBus();
        this.motor = new TalonFX(constants.motorId(), canBus.toPhoenix6CANBus());

        this.rollerPosition = motor.getPosition(false);
        this.rollerVelocity = motor.getVelocity(false);
        this.rollerVoltage = motor.getMotorVoltage(false);
        this.rollerTorqueCurrent = motor.getTorqueCurrent(false);
        this.rollerDeviceTemp = motor.getDeviceTemp(false);

        this.voltageOut = new VoltageOut(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

        RefreshAll.add(
                canBus,
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
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(3.2198)
                .withKV(0.068888)
                .withKA(0.15632)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKP(5.5)
                .withKD(0);
        motorConfig.CurrentLimits.StatorCurrentLimit = 70;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 70;
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

    @Override
    public void toRollersTorqueCurrent(final double torqueCurrentAmps) {
        motor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }

    @Override
    public void toRollersVelocity(final double velocityRotsPerSec) {
        motor.setControl(velocityTorqueCurrentFOC.withVelocity(velocityRotsPerSec));
    }
}
