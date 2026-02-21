package frc.robot.subsystems.intake.slide;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
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
    private final StatusSignal<Angle> differentialPosition;

    private final PositionVoltage averagePositionVoltage;
    private final PositionVoltage differentialPositionVoltage;
    private final VoltageOut voltageOut;

    public IntakeSlideIOReal(final HardwareConstants.IntakeSlideConstants constants) {
        this.averagePositionVoltage = new PositionVoltage(0).withSlot(0);
        this.differentialPositionVoltage = new PositionVoltage(0).withSlot(1);
        this.voltageOut = new VoltageOut(0);

        final TalonFXConfiguration masterMotorConfig = new TalonFXConfiguration();
        //Average Slot
        masterMotorConfig.Slot0 = new Slot0Configs()
                .withKP(9)
                .withKD(3);
        //Diff Slot
        masterMotorConfig.Slot1 = new Slot1Configs()
                .withKP(0.01);
        //Hold Slot
        masterMotorConfig.Slot2 = new Slot2Configs()
                .withKP(0.1);
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
        masterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        masterMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        masterMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        masterMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        final TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        final DifferentialMotorConstants<TalonFXConfiguration> diffConstants =
                new DifferentialMotorConstants<TalonFXConfiguration>()
                        .withCANBusName(constants.CANBus().name())
                        .withLeaderId(constants.masterMotorID())
                        .withFollowerId(constants.followerMotorID())
                        .withAlignment(MotorAlignmentValue.Opposed)
                        //TODO: Ratio might be wrong
                        .withSensorToDifferentialRatio(constants.slideGearing())
                        .withClosedLoopRate(100)
                        .withLeaderInitialConfigs(masterMotorConfig)
                        .withFollowerInitialConfigs(followerConfig)
                        .withFollowerUsesCommonLeaderConfigs(true);

        this.diffMechanism = new DifferentialMechanism<>(TalonFX::new, diffConstants);

        this.masterPosition = diffMechanism.getLeader().getPosition(false);
        this.masterVelocity = diffMechanism.getLeader().getVelocity(false);
        this.masterVoltage = diffMechanism.getLeader().getMotorVoltage(false);
        this.masterTorqueCurrent = diffMechanism.getLeader().getTorqueCurrent(false);
        this.masterDeviceTemp = diffMechanism.getLeader().getDeviceTemp(false);

        this.followerPosition = diffMechanism.getFollower().getPosition(false);
        this.followerVelocity = diffMechanism.getFollower().getVelocity(false);
        this.followerVoltage = diffMechanism.getFollower().getMotorVoltage(false);
        this.followerTorqueCurrent = diffMechanism.getFollower().getTorqueCurrent(false);
        this.followerDeviceTemp = diffMechanism.getFollower().getDeviceTemp(false);

        this.averagePosition = diffMechanism.getAveragePosition(false);
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
                differentialPosition
        );

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
                followerDeviceTemp
        );
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
        inputs.differentialPositionRots = differentialPosition.getValueAsDouble();
    }

    @Override
    public void toSlidePosition(double positionRots) {
        diffMechanism.setControl(averagePositionVoltage.withPosition(positionRots).withSlot(0), differentialPositionVoltage);
    }

    @Override
    public void holdSlidePosition(final double positionRots) {
        diffMechanism.setControl(averagePositionVoltage.withPosition(positionRots).withSlot(2), differentialPositionVoltage);
    }

    @Override
    public void home() {
        diffMechanism.setControl(voltageOut.withOutput(-0.1), voltageOut.withOutput(0));
    }

    @Override
    public void zeroMotors() {
        diffMechanism.setPosition(0);
    }
}
