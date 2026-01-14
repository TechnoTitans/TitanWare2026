package frc.robot.subsystems.superstructure.shooter;

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
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.RefreshAll;

public class ShooterIOReal implements ShooterIO {
    final HardwareConstants.ShooterConstants constants;

    private final TalonFX flywheelMotor;

    private final StatusSignal<Angle> flywheelPosition;
    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<Voltage> flywheelVoltage;
    private final StatusSignal<Current> flywheelTorqueCurrent;
    private final StatusSignal<Temperature> flywheelDeviceTemp;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;



    public ShooterIOReal(final HardwareConstants.ShooterConstants constants) {
        this.constants = constants;

        this.flywheelMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());


        this.flywheelPosition = flywheelMotor.getPosition(false);
        this.flywheelVelocity = flywheelMotor.getVelocity(false);
        this.flywheelVoltage = flywheelMotor.getMotorVoltage(false);
        this.flywheelTorqueCurrent = flywheelMotor.getTorqueCurrent(false);
        this.flywheelDeviceTemp = flywheelMotor.getDeviceTemp(false);

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);

        RefreshAll.add(
                constants.CANBus(),
                flywheelPosition,
                flywheelVelocity,
                flywheelVoltage,
                flywheelTorqueCurrent,
                flywheelDeviceTemp
        );
    }

    @Override
    public void config() {
        final TalonFXConfiguration flywheelConfiguration = new TalonFXConfiguration();
        flywheelConfiguration.Slot0 = new Slot0Configs()
                .withKS(4.0212)
                .withKV(0.40767)
                .withKA(0.22711)
                .withKP(30);
        flywheelConfiguration.CurrentLimits.StatorCurrentLimit = 40;
        flywheelConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        flywheelConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        flywheelConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;
        flywheelConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        flywheelConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flywheelConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        flywheelConfiguration.Feedback.SensorToMechanismRatio = constants.gearing();
        flywheelMotor.getConfigurator().apply(flywheelConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                flywheelPosition,
                flywheelVelocity,
                flywheelVoltage,
                flywheelTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                flywheelDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                flywheelMotor
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelPositionRots = flywheelPosition.getValueAsDouble();
        inputs.flywheelVelocityRotsPerSec = flywheelVelocity.getValueAsDouble();
        inputs.flywheelVoltage = flywheelVoltage.getValueAsDouble();
        inputs.flywheelTorqueCurrentAmps = flywheelTorqueCurrent.getValueAsDouble();
        inputs.flywheelTemp = flywheelDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toFlywheelVelocity(final double velocityRotsPerSec) {
        flywheelMotor.setControl(velocityTorqueCurrentFOC.withVelocity(velocityRotsPerSec));
    }

    @Override
    public void toFlywheelVoltage(final double volts) {
        flywheelMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toFlywheelTorqueCurrent(final double torqueCurrentAmps) {
        flywheelMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }
}
