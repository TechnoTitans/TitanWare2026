package frc.robot.subsystems.intake.slide;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.motors.TalonFXSim;

public class IntakeSlideIOSim implements IntakeSlideIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;

    private final DifferentialMechanism<TalonFX> diffMechanism;

    private final TalonFXSim masterSim;
    private final TalonFXSim followerSim;

    private final StatusSignal<Angle> masterPosition;
    private final StatusSignal<AngularVelocity> masterVelocity;
    private final StatusSignal<Voltage> masterVoltage;
    private final StatusSignal<Current> masterTorqueCurrent;
    private final StatusSignal<Temperature> masterDeviceTemp;

    private final StatusSignal<Angle> followerPosition;
    private final StatusSignal<AngularVelocity> followerVelocity;
    private final StatusSignal<Voltage> followerVoltage;
    private final StatusSignal<Current> followerTorqueCurrent;
    private final StatusSignal<Temperature> followerDeviceTemp;

    private final StatusSignal<Angle> averagePosition;
    private final StatusSignal<AngularVelocity> averageVelocity;
    private final StatusSignal<Angle> differentialPosition;

    private final MotionMagicExpoTorqueCurrentFOC averageMotionMagicExpoTorqueCurrent;
    private final PositionTorqueCurrentFOC averagePositionTorqueCurrentFOC;
    private final PositionTorqueCurrentFOC differentialPositionTorqueCurrentFOC;


    public IntakeSlideIOSim(final HardwareConstants.IntakeSlideConstants constants) {
        this.deltaTime = new DeltaTime(true);

        this.averageMotionMagicExpoTorqueCurrent = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
        this.averagePositionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0).withSlot(1);
        this.differentialPositionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0).withSlot(2);

        final TalonFXConfiguration masterMotorConfig = new TalonFXConfiguration();
        // Average Slot
        masterMotorConfig.Slot0 = new Slot0Configs()
                .withKV(0)
                .withKA(0.1)
                .withKP(200)
                .withKD(10);
        // Hold Slot
        masterMotorConfig.Slot1 = new Slot1Configs()
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKP(10)
                .withKD(0.1);
        // Differential Slot
        masterMotorConfig.Slot2 = new Slot2Configs()
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKP(200)
                .withKD(10);
        masterMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        masterMotorConfig.MotionMagic.MotionMagicExpo_kV = 0.6;
        masterMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.3;
        masterMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        masterMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        masterMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        masterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        masterMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
        masterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        masterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 2.0;
        masterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        masterMotorConfig.Feedback.RotorToSensorRatio = 1;
        masterMotorConfig.Feedback.SensorToMechanismRatio = constants.gearing();
        masterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardLimitRots();
        masterMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        masterMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.reverseLimitRots();
        masterMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        final TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        followerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        followerConfig.Feedback.SensorToMechanismRatio = constants.gearing();

        final DifferentialMotorConstants<TalonFXConfiguration> diffConstants =
                new DifferentialMotorConstants<TalonFXConfiguration>()
                        .withCANBusName(constants.CANBus().name)
                        .withLeaderId(constants.masterMotorID())
                        .withFollowerId(constants.followerMotorID())
                        .withAlignment(MotorAlignmentValue.Opposed)
                        .withSensorToDifferentialRatio(1)
                        .withClosedLoopRate(200)
                        .withLeaderInitialConfigs(masterMotorConfig)
                        .withFollowerInitialConfigs(followerConfig)
                        .withFollowerUsesCommonLeaderConfigs(true);

        this.diffMechanism = new DifferentialMechanism<>(TalonFX::new, diffConstants);

        final TalonFX masterMotor = diffMechanism.getLeader();
        final TalonFX followerMotor = diffMechanism.getFollower();

        masterMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX44);
        followerMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX44);

        final DCMotor motor = DCMotor.getKrakenX44(1);

        final DCMotorSim masterMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        motor,
                        SimConstants.IntakeSlide.MOMENT_OF_INERTIA,
                        constants.gearing()
                ),
                motor
        );

        final DCMotorSim followerMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        motor,
                        SimConstants.IntakeSlide.MOMENT_OF_INERTIA,
                        constants.gearing()
                ),
                motor
        );

        this.masterSim = new TalonFXSim(
                masterMotor,
                constants.gearing(),
                masterMotorSim::update,
                voltage -> masterMotorSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.5)),
                masterMotorSim::getAngularPositionRad,
                masterMotorSim::getAngularVelocityRadPerSec
        );

        this.followerSim = new TalonFXSim(
                followerMotor,
                constants.gearing(),
                followerMotorSim::update,
                voltage -> followerMotorSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.5)),
                followerMotorSim::getAngularPositionRad,
                followerMotorSim::getAngularVelocityRadPerSec
        );

        this.masterPosition = masterMotor.getPosition(false);
        this.masterVelocity = masterMotor.getVelocity(false);
        this.masterVoltage = masterMotor.getMotorVoltage(false);
        this.masterTorqueCurrent = masterMotor.getTorqueCurrent(false);
        this.masterDeviceTemp = masterMotor.getDeviceTemp(false);

        this.followerPosition = followerMotor.getPosition(false);
        this.followerVelocity = followerMotor.getVelocity(false);
        this.followerVoltage = followerMotor.getMotorVoltage(false);
        this.followerTorqueCurrent = followerMotor.getTorqueCurrent(false);
        this.followerDeviceTemp = followerMotor.getDeviceTemp(false);

        this.averagePosition = diffMechanism.getAveragePosition(false);
        this.averageVelocity = diffMechanism.getAverageVelocity(false);
        this.differentialPosition = diffMechanism.getDifferentialPosition(false);

        RefreshAll.add(
                constants.CANBus(),
                masterPosition,
                masterVelocity,
                masterVoltage,
                masterTorqueCurrent,
                masterDeviceTemp,
                followerPosition,
                followerVelocity,
                followerVoltage,
                followerTorqueCurrent,
                followerDeviceTemp,
                averagePosition,
                averageVelocity,
                differentialPosition
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            masterSim.update(dt);
            followerSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d,%d)",
                masterMotor.getDeviceID(),
                followerMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void updateInputs(IntakeSlideIOInputs inputs) {
        inputs.masterPositionRots = masterPosition.getValueAsDouble();
        inputs.masterVelocityRotsPerSec = masterVelocity.getValueAsDouble();
        inputs.masterVoltage = masterVoltage.getValueAsDouble();
        inputs.masterTorqueCurrentAmps = masterTorqueCurrent.getValueAsDouble();
        inputs.masterTempCelsius = masterDeviceTemp.getValueAsDouble();

        inputs.followerPositionRots = followerPosition.getValueAsDouble();
        inputs.followerVelocityRotsPerSec = followerVelocity.getValueAsDouble();
        inputs.followerVoltage = followerVoltage.getValueAsDouble();
        inputs.followerTorqueCurrentAmps = followerTorqueCurrent.getValueAsDouble();
        inputs.followerTempCelsius = followerDeviceTemp.getValueAsDouble();

        inputs.averagePositionRots = averagePosition.getValueAsDouble();
        inputs.averageVelocityRotsPerSec = averageVelocity.getValueAsDouble();
        inputs.differentialPositionRots = differentialPosition.getValueAsDouble();

        inputs.mechanism = diffMechanism.getMechanismState();
    }

    @Override
    public void toSlidePosition(final double positionRots) {
        diffMechanism.setControl(
                averageMotionMagicExpoTorqueCurrent
                        .withPosition(positionRots)
                        .withSlot(0),
                differentialPositionTorqueCurrentFOC
                        .withPosition(0)
                        .withSlot(2)
        );
    }

    @Override
    public void holdSlidePosition(final double positionRots) {
        diffMechanism.setControl(
                averagePositionTorqueCurrentFOC
                        .withPosition(positionRots)
                        .withSlot(2),
                differentialPositionTorqueCurrentFOC
                        .withPosition(0)
                        .withSlot(2)
        );
    }

    @Override
    public void toSlidePositionWithVelocity(final double positionRots, final double velocityRotsPerSec) {
        diffMechanism.setControl(
                averagePositionTorqueCurrentFOC
                        .withPosition(positionRots)
                        .withVelocity(velocityRotsPerSec)
                        .withSlot(0),
                differentialPositionTorqueCurrentFOC
                        .withPosition(0)
                        .withSlot(2)
        );
    }

    @Override
    public void zeroMotors() {
        Phoenix6Utils.tryUntilOk(diffMechanism.getLeader(), () -> diffMechanism.setPosition(0));
    }
}