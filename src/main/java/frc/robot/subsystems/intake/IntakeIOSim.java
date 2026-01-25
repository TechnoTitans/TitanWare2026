package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.feedback.SimCANCoder;
import frc.robot.utils.sim.motors.TalonFXSim;

import java.util.List;

public class IntakeIOSim implements IntakeIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.IntakeConstants constants;

    private final TalonFX rollerMotor;
    private final TalonFX masterSliderMotor;
    private final TalonFX followerSliderMotor;
    private final CANcoder sliderEncoder;

    private final TalonFXSim rollerMotorSim;
    private final TalonFXSim sliderMotorSim;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final Follower follower;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerDeviceTemp;

    private final StatusSignal<Angle> masterSliderPosition;
    private final StatusSignal<AngularVelocity> masterSliderVelocity;
    private final StatusSignal<Voltage> masterSliderVoltage;
    private final StatusSignal<Current> masterSliderTorqueCurrent;
    private final StatusSignal<Temperature> masterSliderDeviceTemp;

    private final StatusSignal<Angle> followerSliderPosition;
    private final StatusSignal<AngularVelocity> followerSliderVelocity;
    private final StatusSignal<Voltage> followerSliderVoltage;
    private final StatusSignal<Current> followerSliderTorqueCurrent;
    private final StatusSignal<Temperature> followerSliderDeviceTemp;

    private final StatusSignal<Angle> sliderCANCoderPosition;
    private final StatusSignal<AngularVelocity> sliderCANCoderVelocity;

    public IntakeIOSim(final HardwareConstants.IntakeConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        this.rollerMotor = new TalonFX(constants.rollerMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.masterSliderMotor = new TalonFX(constants.masterSliderMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.followerSliderMotor = new TalonFX(constants.followerSliderMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.sliderEncoder = new CANcoder(constants.encoderID(), constants.CANBus().toPhoenix6CANBus());

        final DCMotorSim rollerMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    1 / (2 * Math.PI),
                    0.1 / (2 * Math.PI)
            ),
            DCMotor.getKrakenX44Foc(1)
        );

        this.rollerMotorSim = new TalonFXSim(
                rollerMotor,
                constants.rollerGearing(),
                rollerMotorSim::update,
                rollerMotorSim::setInputVoltage,
                rollerMotorSim::getAngularPositionRad,
                rollerMotorSim::getAngularVelocityRadPerSec
        );

        final DCMotorSim sliderSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        2.5 / (2 * Math.PI),
                        0.01 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(1)
        );

        this.sliderMotorSim = new TalonFXSim(
                List.of(masterSliderMotor, followerSliderMotor),
                constants.sliderGearing(),
                sliderSim::update,
                sliderSim::setInputVoltage,
                sliderSim::getAngularPositionRad,
                sliderSim::getAngularVelocityRadPerSec
        );

        this.sliderMotorSim.attachFeedbackSensor(new SimCANCoder(sliderEncoder));

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);
        this.follower = new Follower(masterSliderMotor.getDeviceID(), MotorAlignmentValue.Opposed);

        this.rollerPosition = rollerMotor.getPosition(false);
        this.rollerVelocity = rollerMotor.getVelocity(false);
        this.rollerVoltage = rollerMotor.getMotorVoltage(false);
        this.rollerTorqueCurrent = rollerMotor.getTorqueCurrent(false);
        this.rollerDeviceTemp = rollerMotor.getDeviceTemp(false);

        this.masterSliderPosition = masterSliderMotor.getPosition(false);
        this.masterSliderVelocity = masterSliderMotor.getVelocity(false);
        this.masterSliderVoltage = masterSliderMotor.getMotorVoltage(false);
        this.masterSliderTorqueCurrent = masterSliderMotor.getTorqueCurrent(false);
        this.masterSliderDeviceTemp = masterSliderMotor.getDeviceTemp(false);

        this.followerSliderPosition = followerSliderMotor.getPosition(false);
        this.followerSliderVelocity = followerSliderMotor.getVelocity(false);
        this.followerSliderVoltage = followerSliderMotor.getMotorVoltage(false);
        this.followerSliderTorqueCurrent = followerSliderMotor.getTorqueCurrent(false);
        this.followerSliderDeviceTemp = followerSliderMotor.getDeviceTemp(false);

        this.sliderCANCoderPosition = sliderEncoder.getPosition(false);
        this.sliderCANCoderVelocity = sliderEncoder.getVelocity(false);

        RefreshAll.add(
                constants.CANBus(),
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                rollerDeviceTemp,
                masterSliderPosition,
                masterSliderVelocity,
                masterSliderVoltage,
                masterSliderTorqueCurrent,
                masterSliderDeviceTemp,
                followerSliderPosition,
                followerSliderVelocity,
                followerSliderVoltage,
                followerSliderTorqueCurrent,
                followerSliderDeviceTemp,
                sliderCANCoderPosition,
                sliderCANCoderVelocity
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            this.rollerMotorSim.update(dt);
            this.sliderMotorSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d), (%d), (%d)",
                rollerMotor.getDeviceID(),
                masterSliderMotor.getDeviceID(),
                followerSliderMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        final TalonFXConfiguration rollerConfigurator = new TalonFXConfiguration();
        rollerConfigurator.Slot0 = new Slot0Configs()
                .withKP(5);
        rollerConfigurator.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        rollerConfigurator.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        rollerConfigurator.CurrentLimits.StatorCurrentLimit = 60;
        rollerConfigurator.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfigurator.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerConfigurator.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerConfigurator.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rollerConfigurator.Feedback.SensorToMechanismRatio = constants.rollerGearing();
        rollerMotor.getConfigurator().apply(rollerConfigurator);

        final CANcoderConfiguration sliderCANCoderConfig = new CANcoderConfiguration();
        sliderCANCoderConfig.MagnetSensor.MagnetOffset = constants.encoderOffset();
        sliderCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        sliderEncoder.getConfigurator().apply(sliderCANCoderConfig);

        final TalonFXConfiguration sliderMotorConfig = new TalonFXConfiguration();
        sliderMotorConfig.Slot0 = new Slot0Configs()
                .withKP(30);
        sliderMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        sliderMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        sliderMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        sliderMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        sliderMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        sliderMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        sliderMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        sliderMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        sliderMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        sliderMotorConfig.Feedback.FeedbackRemoteSensorID = sliderEncoder.getDeviceID();
        sliderMotorConfig.Feedback.RotorToSensorRatio = constants.sliderGearing();
        sliderMotorConfig.Feedback.SensorToMechanismRatio = 1;
        sliderMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        sliderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        sliderMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        sliderMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        sliderMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        sliderMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        masterSliderMotor.getConfigurator().apply(sliderMotorConfig);
        sliderMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerSliderMotor.getConfigurator().apply(sliderMotorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                masterSliderPosition,
                masterSliderVelocity,
                masterSliderVoltage,
                masterSliderTorqueCurrent,
                followerSliderPosition,
                followerSliderVelocity,
                followerSliderVoltage,
                followerSliderTorqueCurrent,
                sliderCANCoderPosition,
                sliderCANCoderVelocity
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                rollerDeviceTemp,
                masterSliderDeviceTemp,
                followerSliderDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                rollerMotor,
                masterSliderMotor,
                followerSliderMotor,
                sliderEncoder
        );

        rollerMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        sliderEncoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        masterSliderMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        followerSliderMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerPositionRots = rollerPosition.getValueAsDouble();
        inputs.rollerVelocityRotsPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerDeviceTemp.getValueAsDouble();

        inputs.masterSliderPositionRots = masterSliderPosition.getValueAsDouble();
        inputs.masterSliderVelocityRotsPerSec = masterSliderVelocity.getValueAsDouble();
        inputs.masterSliderVoltage = masterSliderVoltage.getValueAsDouble();
        inputs.masterSliderTorqueCurrentAmps = masterSliderTorqueCurrent.getValueAsDouble();
        inputs.masterSliderTempCelsius = masterSliderDeviceTemp.getValueAsDouble();

        inputs.followerSliderPositionRots = followerSliderPosition.getValueAsDouble();
        inputs.followerSliderVelocityRotsPerSec = followerSliderVelocity.getValueAsDouble();
        inputs.followerSliderVoltage = followerSliderVoltage.getValueAsDouble();
        inputs.followerSliderTorqueCurrentAmps = followerSliderTorqueCurrent.getValueAsDouble();
        inputs.followerSliderTempCelsius = followerSliderDeviceTemp.getValueAsDouble();

        inputs.encoderPositionRots = sliderCANCoderPosition.getValueAsDouble();
        inputs.encoderVelocityRotsPerSec = sliderCANCoderVelocity.getValueAsDouble();
    }

    @Override
    public void toRollerVelocity(final double velocityRotsPerSec) {
        rollerMotor.setControl(velocityTorqueCurrentFOC.withVelocity(velocityRotsPerSec));
    }

    @Override
    public void toRollerVoltage(final double volts) {
        rollerMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toRollerTorqueCurrent(final double torqueCurrentAmps) {
        rollerMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }

    @Override
    public void toSliderPosition(final double sliderPositionRots) {
        masterSliderMotor.setControl(motionMagicExpoVoltage.withPosition(sliderPositionRots));
        followerSliderMotor.setControl(follower);
    }

    @Override
    public void toSliderVoltage(final double sliderVolts) {
        masterSliderMotor.setControl(voltageOut.withOutput(sliderVolts));
        followerSliderMotor.setControl(follower);
    }

    @Override
    public void toSliderTorqueCurrent(final double sliderTorqueCurrentAmps) {
        masterSliderMotor.setControl(torqueCurrentFOC.withOutput(sliderTorqueCurrentAmps));
        followerSliderMotor.setControl(follower);
    }
}
