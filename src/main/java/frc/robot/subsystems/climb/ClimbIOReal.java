package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class ClimbIOReal implements ClimbIO{
    private final HardwareConstants.ClimbConstants constants;

    private final TalonFX climbMotor;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final TorqueCurrentFOC torqueCurrentFOC;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    public ClimbIOReal(final HardwareConstants.ClimbConstants constants) {
        this.constants = constants;

        this.climbMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);

        this.motorPosition = climbMotor.getPosition(false);
        this.motorVelocity = climbMotor.getVelocity(false);
        this.motorVoltage = climbMotor.getMotorVoltage(false);
        this.motorTorqueCurrent = climbMotor.getTorqueCurrent(false);
        this.motorDeviceTemp = climbMotor.getDeviceTemp(false);

        RefreshAll.add(
                HardwareConstants.CANBus.CANIVORE,
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
                .withKS(0.037707)
                .withKG(0.52705)
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKV(0.54587)
                .withKA(0.012141)
                .withKP(15)
                .withKD(1);
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorConfiguration.MotionMagic.MotionMagicExpo_kV = 0.54587;
        motorConfiguration.MotionMagic.MotionMagicExpo_kA = 0.1;
        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 70;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -70;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 70;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.climbGearing();
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        climbMotor.getConfigurator().apply(motorConfiguration);

        motorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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
    public void setPosition(final double positionRots) {
        Phoenix6Utils.reportIfNotOk(climbMotor, climbMotor.setPosition(positionRots));
    }

    @Override
    public void toPosition(final double positionRots) {
        climbMotor.setControl(motionMagicExpoVoltage.withPosition(positionRots));
    }

    @Override
    public void toTorqueCurrent(final double torqueCurrentAmps) {
        climbMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }
}

