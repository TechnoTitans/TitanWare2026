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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.RefreshAll;

public class IntakeSlideIOReal implements IntakeSlideIO {
    private final DifferentialMechanism<TalonFX> diffMechanism;

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

    public IntakeSlideIOReal(final HardwareConstants.IntakeSlideConstants constants) {
        this.averageMotionMagicExpoTorqueCurrent = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
        this.averagePositionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0).withSlot(1);
        this.differentialPositionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0).withSlot(2);

        final TalonFXConfiguration masterMotorConfig = new TalonFXConfiguration();
        // Average Slot
        masterMotorConfig.Slot0 = new Slot0Configs()
                .withKV(5)
                .withKP(20)
                .withKD(0.5);
        // Hold Slot
        masterMotorConfig.Slot1 = new Slot1Configs()
                .withKP(1)
                .withKD(0.1);
        // Differential Slot
        masterMotorConfig.Slot2 = new Slot2Configs()
                .withKP(250)
                .withKD(0.1);
        masterMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        masterMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        masterMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        masterMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        masterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        masterMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        masterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        masterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        masterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        masterMotorConfig.Feedback.RotorToSensorRatio = 1;
        masterMotorConfig.Feedback.SensorToMechanismRatio = constants.slideGearing();
        masterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        masterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        masterMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        masterMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        masterMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        final TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        followerConfig.Feedback.SensorToMechanismRatio = constants.slideGearing();

        final DifferentialMotorConstants<TalonFXConfiguration> diffConstants =
                new DifferentialMotorConstants<TalonFXConfiguration>()
                        .withCANBusName(constants.CANBus().name())
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
    }

    @Override
    public void updateInputs(final IntakeSlideIOInputs inputs) {
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
    public void toSlidePosition(double positionRots) {
        diffMechanism.setControl(
                averageMotionMagicExpoTorqueCurrent.withPosition(positionRots).withSlot(0),
                differentialPositionTorqueCurrentFOC.withPosition(0).withSlot(2)
        );
    }

    @Override
    public void holdSlidePosition(final double positionRots) {
        diffMechanism.setControl(
                averagePositionTorqueCurrentFOC.withPosition(positionRots).withSlot(2),
                differentialPositionTorqueCurrentFOC.withPosition(0).withSlot(2)
        );
    }

    @Override
    public void toSlidePositionUnprofiled(double positionRots, double velocityRotsPerSec) {
        diffMechanism.setControl(
                averagePositionTorqueCurrentFOC
                        .withPosition(positionRots).withVelocity(velocityRotsPerSec).withSlot(0),
                differentialPositionTorqueCurrentFOC.withPosition(0).withSlot(2)
        );
    }

    @Override
    public void zeroMotors() {
        diffMechanism.setPosition(0);
    }
}