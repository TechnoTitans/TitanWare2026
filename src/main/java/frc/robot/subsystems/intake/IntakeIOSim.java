package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.MoreDCMotor;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.feedback.SimCANCoder;
import frc.robot.utils.sim.motors.TalonFXSim;

public class IntakeIOSim implements IntakeIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.IntakeConstants constants;

    private final TalonFX rollerMotor;
    private final TalonFX sliderMotor;
    private final CANcoder sliderEncoder;

    private final TalonFXSim rollerMotorSim;
    private final TalonFXSim sliderMotorSim;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerDeviceTemp;

    private final StatusSignal<Angle> sliderPosition;
    private final StatusSignal<AngularVelocity> sliderVelocity;
    private final StatusSignal<Voltage> sliderVoltage;
    private final StatusSignal<Current> sliderTorqueCurrent;
    private final StatusSignal<Temperature> sliderDeviceTemp;
    private final StatusSignal<Angle> sliderCANCoderPosition;
    private final StatusSignal<AngularVelocity> sliderCANCoderVelocity;

    public IntakeIOSim(final HardwareConstants.IntakeConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        this.rollerMotor = new TalonFX(constants.rollerMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.sliderMotor = new TalonFX(constants.sliderMotorID(), constants.CANBus().toPhoenix6CANBus());
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
                        4 / (2 * Math.PI),
                        0.04 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(0)
        );

        this.sliderMotorSim = new TalonFXSim(
                sliderMotor,
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

        this.rollerPosition = rollerMotor.getPosition(false);
        this.rollerVelocity = rollerMotor.getVelocity(false);
        this.rollerVoltage = rollerMotor.getMotorVoltage(false);
        this.rollerTorqueCurrent = rollerMotor.getTorqueCurrent(false);
        this.rollerDeviceTemp = rollerMotor.getDeviceTemp(false);

        this.sliderPosition = sliderMotor.getPosition(false);
        this.sliderVelocity = sliderMotor.getVelocity(false);
        this.sliderVoltage = sliderMotor.getMotorVoltage(false);
        this.sliderTorqueCurrent = sliderMotor.getTorqueCurrent(false);
        this.sliderDeviceTemp = sliderMotor.getDeviceTemp(false);
        this.sliderCANCoderPosition = sliderEncoder.getPosition(false);
        this.sliderCANCoderVelocity = sliderEncoder.getVelocity(false);

        RefreshAll.add(
                constants.CANBus(),
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                rollerDeviceTemp,
                sliderPosition,
                sliderVelocity,
                sliderVoltage,
                sliderTorqueCurrent,
                sliderDeviceTemp,
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
                "SimUpdate(%d), (%d)",
                rollerMotor.getDeviceID(),
                sliderMotor.getDeviceID()
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
        sliderMotor.getConfigurator().apply(sliderMotorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                sliderPosition,
                sliderVelocity,
                sliderVoltage,
                sliderTorqueCurrent,
                sliderCANCoderPosition,
                sliderCANCoderVelocity
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                rollerDeviceTemp,
                sliderDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                rollerMotor,
                sliderMotor,
                sliderEncoder
        );

        rollerMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        sliderEncoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        sliderMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerPositionRots = rollerPosition.getValueAsDouble();
        inputs.rollerVelocityRotsPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerDeviceTemp.getValueAsDouble();

        inputs.sliderPositionRots = sliderPosition.getValueAsDouble();
        inputs.sliderVelocityRotsPerSec = sliderVelocity.getValueAsDouble();
        inputs.sliderVoltage = sliderVoltage.getValueAsDouble();
        inputs.sliderTorqueCurrentAmps = sliderTorqueCurrent.getValueAsDouble();
        inputs.sliderTempCelsius = sliderDeviceTemp.getValueAsDouble();

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
        sliderMotor.setControl(motionMagicExpoVoltage.withPosition(sliderPositionRots));
    }

    @Override
    public void toSliderVoltage(final double sliderVolts) {
        sliderMotor.setControl(voltageOut.withOutput(sliderVolts));
    }

    @Override
    public void toSliderTorqueCurrent(final double sliderTorqueCurrentAmps) {
        sliderMotor.setControl(torqueCurrentFOC.withOutput(sliderTorqueCurrentAmps));
    }
}
