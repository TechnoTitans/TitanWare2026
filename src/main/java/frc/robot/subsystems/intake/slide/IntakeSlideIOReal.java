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
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
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

        final TalonFXConfiguration masterConfiguration = new TalonFXConfiguration();
        //Avg Stiff
        masterConfiguration.Slot0 = new Slot0Configs()
                .withKS(1)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKP(13)
                .withKD(0.12);
        //Avg / Diff Squishy
        masterConfiguration.Slot1 = new Slot1Configs()
                .withKS(1)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKP(8) //15
                .withKD(0.12);
        //Diff Stiff
        masterConfiguration.Slot2 = new Slot2Configs()
                .withKP(150)
                .withKD(0);
        masterConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        masterConfiguration.MotionMagic.MotionMagicExpo_kV = 0.6;
        masterConfiguration.MotionMagic.MotionMagicExpo_kA = 0.3;
        masterConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        masterConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        masterConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        masterConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        masterConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
        masterConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        masterConfiguration.CurrentLimits.SupplyCurrentLowerTime = 2.0;
        masterConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        masterConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        masterConfiguration.Feedback.SensorToMechanismRatio = constants.gearing();
        masterConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        masterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        masterConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardLimitRots();
        masterConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        masterConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.reverseLimitRots();
        masterConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        final TalonFXConfiguration followerConfiguration = new TalonFXConfiguration();
        followerConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        followerConfiguration.Feedback.SensorToMechanismRatio = constants.gearing();

        final DifferentialMotorConstants<TalonFXConfiguration> differentialConstants =
                new DifferentialMotorConstants<>();
        differentialConstants.LeaderId = constants.masterMotorID();
        differentialConstants.FollowerId = constants.followerMotorID();
        differentialConstants.CANBusName = constants.CANBus().name;
        differentialConstants.Alignment = MotorAlignmentValue.Opposed;
        differentialConstants.SensorToDifferentialRatio = 1;
        differentialConstants.ClosedLoopRate = 200;
        differentialConstants.LeaderInitialConfigs = masterConfiguration;
        differentialConstants.FollowerInitialConfigs = followerConfiguration;
        differentialConstants.FollowerUsesCommonLeaderConfigs = true;

        this.diffMechanism = new DifferentialMechanism<>(TalonFX::new, differentialConstants);

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