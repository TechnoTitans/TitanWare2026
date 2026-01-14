package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.MoreDCMotor;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.motors.TalonFXSim;

public class HoodIOSim implements HoodIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.HoodConstants constants;

    private final TalonFX hoodMotor;
    private final TalonFXSim hoodTalonFXSim;
    private final TalonFXConfiguration motorConfig;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodVoltage;
    private final StatusSignal<Current> hoodTorqueCurrent;
    private final StatusSignal<Temperature> hoodDeviceTemp;

    private final PositionVoltage positionVoltage;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    public HoodIOSim(final HardwareConstants.HoodConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final double zeroedPositionToHorizontalRads = SimConstants.Hood.ZEROED_POSITION_TO_HORIZONTAL.getRadians();
        final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
                LinearSystemId.identifyPositionSystem(
                        5 / (2d * Math.PI),
                        0.04 / (2d * Math.PI)
                ),
                MoreDCMotor.getKrakenX44(1),
                constants.hoodGearing(),
                SimConstants.Hood.LENGTH_METERS,
                Units.rotationsToRadians(constants.hoodLowerLimitRots()) + zeroedPositionToHorizontalRads,
                Units.rotationsToRadians(constants.hoodUpperLimitRots()) + zeroedPositionToHorizontalRads,
                true,
                SimConstants.Hood.STARTING_ANGLE.getRadians()
        );

        this.hoodMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());
        this.motorConfig = new TalonFXConfiguration();

        this.hoodTalonFXSim = new TalonFXSim(
                hoodMotor,
                constants.hoodGearing(),
                hoodSim::update,
                hoodSim::setInputVoltage,
                () -> hoodSim.getAngleRads() - zeroedPositionToHorizontalRads,
                hoodSim::getVelocityRadPerSec
        );

        this.hoodPosition = hoodMotor.getPosition(false);
        this.hoodVelocity = hoodMotor.getVelocity(false);
        this.hoodVoltage = hoodMotor.getMotorVoltage(false);
        this.hoodTorqueCurrent = hoodMotor.getTorqueCurrent(false);
        this.hoodDeviceTemp = hoodMotor.getDeviceTemp(false);

        this.positionVoltage = new PositionVoltage(0);
        this.voltageOut = new VoltageOut(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);

        RefreshAll.add(
                constants.CANBus(),
                hoodPosition,
                hoodVelocity,
                hoodVoltage,
                hoodTorqueCurrent,
                hoodDeviceTemp
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            hoodTalonFXSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d)",
                hoodMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        motorConfig.Slot0 = new Slot0Configs()
                .withKG(0.1)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(10);
        motorConfig.CurrentLimits.StatorCurrentLimit = 50;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.hoodGearing();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.hoodUpperLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.hoodLowerLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodMotor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                hoodPosition,
                hoodVelocity,
                hoodVoltage,
                hoodTorqueCurrent
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                hoodDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                4,
                hoodMotor
        );
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodPositionRots = hoodPosition.getValueAsDouble();
        inputs.hoodVelocityRotsPerSec = hoodVelocity.getValueAsDouble();
        inputs.hoodVoltage = hoodVoltage.getValueAsDouble();
        inputs.hoodTorqueCurrentAmps = hoodTorqueCurrent.getValueAsDouble();
        inputs.hoodTempCelsius = hoodDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toHoodPosition(final double positionRots) {
        hoodMotor.setControl(positionVoltage.withPosition(positionRots));
    }

    @Override
    public void toHoodContinuousPosition(final double positionRots) {
        hoodMotor.setControl(positionVoltage.withPosition(positionRots));
    }

    @Override
    public void toHoodVoltage(final double volts) {
        hoodMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toHoodTorqueCurrent(final double torqueCurrent) {
        hoodMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrent));
    }

    @Override
    public void home() {
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 1;
        hoodMotor.setControl(voltageOut.withOutput(-0.1));
    }

    @Override
    public void zeroMotor() {
        hoodMotor.setPosition(0);
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.hoodUpperLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        toHoodPosition(0);
    }
}
