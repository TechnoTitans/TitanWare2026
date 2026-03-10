package frc.robot.subsystems.intake.slide;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.motors.TalonFXSim;

public class IntakeSlideIOSim implements IntakeSlideIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final TalonFXSim masterSim;
    private final TalonFXSim followerSim;

    private final DifferentialMechanism<TalonFX> differentialMechanism;

    private final MotionMagicExpoTorqueCurrentFOC avg_mmExpoTorqueCurrentFOC;
    private final PositionTorqueCurrentFOC avg_positionTorqueCurrentFOC;
    private final PositionTorqueCurrentFOC diff_positionTorqueCurrentFOC;
    private final TorqueCurrentFOC avg_torqueCurrentFOC;

    private final StatusSignal<Angle> averagePosition;
    private final StatusSignal<AngularVelocity> averageVelocity;
    private final StatusSignal<Angle> differentialPosition;
    private final StatusSignal<AngularVelocity> differentialVelocity;

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

    public IntakeSlideIOSim(final HardwareConstants.IntakeSlideConstants constants) {
        this.deltaTime = new DeltaTime(true);

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();

        this.avg_mmExpoTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);
        this.avg_positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);
        this.diff_positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);
        this.avg_torqueCurrentFOC = new TorqueCurrentFOC(0);

        final TalonFXConfiguration masterConfiguration = new TalonFXConfiguration();
        //Avg Stiff
        masterConfiguration.Slot0 = new Slot0Configs() //TODO: Tune
                .withKS(3)
                .withKP(20)
                .withKD(0.5);
        //Avg / Diff Squishy
        masterConfiguration.Slot1 = new Slot1Configs()
                .withKS(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKA(0)
                .withKP(15)
                .withKD(0);
        //Diff Stiff
        masterConfiguration.Slot2 = new Slot2Configs()
                .withKP(250)
                .withKD(0.1);
        masterConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        masterConfiguration.MotionMagic.MotionMagicExpo_kV = 0.6;
        masterConfiguration.MotionMagic.MotionMagicExpo_kA = 0.3;
        masterConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        masterConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        masterConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        masterConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        masterConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        masterConfiguration.Feedback.SensorToMechanismRatio = constants.averageAxisGearing();
        masterConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardLimitRots();
        masterConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        masterConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.reverseLimitRots();
        masterConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        final TalonFXConfiguration followerConfiguration = new TalonFXConfiguration();
        followerConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        followerConfiguration.Feedback.SensorToMechanismRatio = constants.averageAxisGearing();

        final DifferentialMotorConstants<TalonFXConfiguration> differentialConstants =
                new DifferentialMotorConstants<>();
        differentialConstants.CANBusName = p6Bus.getName();
        differentialConstants.LeaderId = constants.masterMotorId();
        differentialConstants.FollowerId = constants.followerMotorId();
        differentialConstants.Alignment = MotorAlignmentValue.Opposed;
        differentialConstants.SensorToDifferentialRatio = constants.differentialAxisGearing();
        differentialConstants.ClosedLoopRate = 200;
        differentialConstants.LeaderInitialConfigs = masterConfiguration;
        differentialConstants.FollowerInitialConfigs = followerConfiguration;
        differentialConstants.FollowerUsesCommonLeaderConfigs = true;

        this.differentialMechanism = new DifferentialMechanism<>(TalonFX::new, differentialConstants);

        this.averagePosition = differentialMechanism.getAveragePosition(false);
        this.averageVelocity = differentialMechanism.getAverageVelocity(false);
        this.differentialPosition = differentialMechanism.getDifferentialPosition(false);
        this.differentialVelocity = differentialMechanism.getDifferentialVelocity(false);

        final TalonFX masterMotor = differentialMechanism.getLeader();
        {
            final DCMotor dcMotor = DCMotor.getKrakenX44Foc(1);
            final DCMotorSim dcMotorSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(dcMotor, 0.0058, constants.averageAxisGearing()),
                    dcMotor
            );
            this.masterSim = new TalonFXSim(
                    masterMotor,
                    constants.averageAxisGearing(),
                    dcMotorSim::update,
                    voltage -> dcMotorSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.5)),
                    dcMotorSim::getAngularPositionRad,
                    dcMotorSim::getAngularVelocityRadPerSec
            );

            final TalonFXSimState simState = masterMotor.getSimState();
            simState.Orientation = ChassisReference.CounterClockwise_Positive;
            simState.setMotorType(TalonFXSimState.MotorType.KrakenX44);
        }

        this.masterPosition = masterMotor.getPosition(false);
        this.masterVelocity = masterMotor.getVelocity(false);
        this.masterVoltage = masterMotor.getMotorVoltage(false);
        this.masterTorqueCurrent = masterMotor.getTorqueCurrent(false);
        this.masterDeviceTemp = masterMotor.getDeviceTemp(false);

        final TalonFX followerMotor = differentialMechanism.getFollower();
        {
            final DCMotor dcMotor = DCMotor.getKrakenX44Foc(1);
            final DCMotorSim dcMotorSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(dcMotor, 0.0058, constants.averageAxisGearing()),
                    dcMotor
            );
            this.followerSim = new TalonFXSim(
                    followerMotor,
                    constants.averageAxisGearing(),
                    dcMotorSim::update,
                    voltage -> dcMotorSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.25)),
                    dcMotorSim::getAngularPositionRad,
                    dcMotorSim::getAngularVelocityRadPerSec
            );

            final TalonFXSimState simState = followerMotor.getSimState();
            simState.Orientation = ChassisReference.CounterClockwise_Positive;
            simState.setMotorType(TalonFXSimState.MotorType.KrakenX44);
        }

        this.followerPosition = followerMotor.getPosition(false);
        this.followerVelocity = followerMotor.getVelocity(false);
        this.followerVoltage = followerMotor.getMotorVoltage(false);
        this.followerTorqueCurrent = followerMotor.getTorqueCurrent(false);
        this.followerDeviceTemp = followerMotor.getDeviceTemp(false);

        RefreshAll.add(
                bus,
                averagePosition,
                averageVelocity,
                differentialPosition,
                differentialVelocity,
                masterPosition,
                masterVelocity,
                masterVoltage,
                masterTorqueCurrent,
                masterDeviceTemp,
                followerPosition,
                followerVelocity,
                followerVoltage,
                followerTorqueCurrent,
                followerDeviceTemp
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
    public void updateInputs(final IntakeSlideIOInputs inputs) {
        differentialMechanism.periodic();

        inputs.slideState = differentialMechanism.getMechanismState();

        inputs.slideAveragePositionRots = averagePosition.getValueAsDouble();
        inputs.slideAverageVelocityRotsPerSec = averageVelocity.getValueAsDouble();
        inputs.slideDifferentialPositionRots = differentialPosition.getValueAsDouble();
        inputs.slideDifferentialVelocityRotsPerSec = differentialVelocity.getValueAsDouble();

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
    }

    @Override
    public void toSlidePosition(final double slidePositionRots) {
        differentialMechanism.setControl(
                avg_mmExpoTorqueCurrentFOC
                        .withPosition(slidePositionRots)
                        .withSlot(0),
                diff_positionTorqueCurrentFOC
                        .withPosition(0)
                        .withSlot(2)
        );
    }

    @Override
    public void holdSlidePosition(final double slidePositionRots) {
        differentialMechanism.setControl(
                avg_positionTorqueCurrentFOC
                        .withPosition(slidePositionRots)
                        .withSlot(1),
                diff_positionTorqueCurrentFOC
                        .withPosition(0)
                        .withSlot(1)
        );
    }

    @Override
    public void toSlidePositionVelocity(final double slidePositionRots, final double slideVelocityRots) {
        differentialMechanism.setControl(
                avg_positionTorqueCurrentFOC
                        .withPosition(slidePositionRots)
                        .withVelocity(slideVelocityRots)
                        .withSlot(0),
                diff_positionTorqueCurrentFOC
                        .withPosition(0)
                        .withSlot(2)
        );
    }

    @Override
    public void toSlideTorqueCurrent(final double slideTorqueCurrentAmps) {
        differentialMechanism.setControl(
                avg_torqueCurrentFOC
                        .withOutput(slideTorqueCurrentAmps),
                diff_positionTorqueCurrentFOC
                        .withPosition(0)
                        .withSlot(1)
        );
    }
}
