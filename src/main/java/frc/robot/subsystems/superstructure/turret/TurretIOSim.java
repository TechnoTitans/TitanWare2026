package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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
import frc.robot.constants.SimConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.motors.TalonFXSim;

public class TurretIOSim implements TurretIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private static final int CAPACITY = 40;
    private int fuelStored = 0;

    private final DeltaTime deltaTime;
    private final HardwareConstants.TurretConstants constants;

    private final TalonFX turretMotor;
    private final CANcoder leftEncoder;
    private final CANcoder rightEncoder;

    private final TalonFXSim turretTalonFXSim;

    private final StatusSignal<Angle> turretPosition;
    private final StatusSignal<AngularVelocity> turretVelocity;
    private final StatusSignal<Voltage> turretVoltage;
    private final StatusSignal<Current> turretTorqueCurrent;
    private final StatusSignal<Temperature> turretDeviceTemp;

    private final StatusSignal<Angle> leftEncoderPosition;
    private final StatusSignal<Angle> rightEncoderPosition;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final PositionVoltage positionVoltage;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    public TurretIOSim(final HardwareConstants.TurretConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        this.turretMotor = new TalonFX(constants.turretMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.leftEncoder = new CANcoder(constants.leftEncoderID(), constants.CANBus().toPhoenix6CANBus());
        this.rightEncoder = new CANcoder(constants.rightEncoderID(), constants.CANBus().toPhoenix6CANBus());

        final DCMotorSim turretSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60Foc(1),
                        SimConstants.Turret.MOMENT_OF_INERTIA,
                        constants.turretToMechanismGearing()
                ),
                DCMotor.getKrakenX60Foc(1)
        );

        this.turretTalonFXSim = new TalonFXSim(
                turretMotor,
                constants.turretToMechanismGearing(),
                turretSim::update,
                turretSim::setInputVoltage,
                turretSim::getAngularPositionRad,
                turretSim::getAngularVelocityRadPerSec
        );

        this.turretPosition = turretMotor.getPosition(false);
        this.turretVelocity = turretMotor.getVelocity(false);
        this.turretVoltage = turretMotor.getMotorVoltage(false);
        this.turretTorqueCurrent = turretMotor.getTorqueCurrent(false);
        this.turretDeviceTemp = turretMotor.getDeviceTemp(false);

        this.leftEncoderPosition = leftEncoder.getPosition(true);
        this.rightEncoderPosition = rightEncoder.getPosition(true);

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.positionVoltage = new PositionVoltage(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        RefreshAll.add(
                constants.CANBus(),
                turretPosition,
                turretVelocity,
                turretVoltage,
                turretTorqueCurrent,
                turretDeviceTemp,
                leftEncoderPosition,
                rightEncoderPosition
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            turretTalonFXSim.update(dt);


            leftEncoder.getSimState().setRawPosition(turretTalonFXSim.getAngularPositionRots() *
                    constants.turretTooth() / constants.leftEncoderGearing());

            rightEncoder.getSimState().setRawPosition(leftEncoder.getPosition().getValueAsDouble() *
                    constants.leftEncoderGearing() / constants.rightEncoderGearing());
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d)",
                turretMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(0.1)
                .withKV(5)
                .withKP(60)
                .withKD(0.1);
        motorConfig.Slot1 = new Slot1Configs()
                .withKS(0.1)
                .withKP(40)
                .withKD(10);
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorConfig.MotionMagic.MotionMagicExpo_kV = 0;
        motorConfig.MotionMagic.MotionMagicExpo_kA = 0;
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.turretToMechanismGearing();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretMotor.getConfigurator().apply(motorConfig);

        final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = constants.leftEncoderOffset();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        leftEncoder.getConfigurator().apply(encoderConfig);

        encoderConfig.MagnetSensor.MagnetOffset = constants.rightEncoderOffset();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        rightEncoder.getConfigurator().apply(encoderConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                turretPosition,
                turretVelocity,
                turretVoltage,
                turretTorqueCurrent,
                leftEncoderPosition,
                rightEncoderPosition
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                turretDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                4,
                turretMotor,
                leftEncoder,
                rightEncoder
        );

        turretMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        leftEncoder.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        rightEncoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public void updateInputs(final TurretIOInputs inputs) {
        inputs.turretPositionRots = turretPosition.getValueAsDouble();
        inputs.turretVelocityRotsPerSec = turretVelocity.getValueAsDouble();
        inputs.turretVoltage = turretVoltage.getValueAsDouble();
        inputs.turretTorqueCurrentAmps = turretTorqueCurrent.getValueAsDouble();
        inputs.turretTempCelsius = turretDeviceTemp.getValueAsDouble();

        inputs.leftPositionRots = leftEncoderPosition.getValueAsDouble();
        inputs.rightPositionRots = rightEncoderPosition.getValueAsDouble();
    }

    @Override
    public void toTurretContinuousPosition(final double positionRots, final double velocityRotsPerSec) {
        turretMotor.setControl(positionVoltage.withPosition(positionRots).withVelocity(-velocityRotsPerSec));
    }

    @Override
    public void toTurretPosition(final double positionRots) {
        turretMotor.setControl(motionMagicExpoVoltage.withPosition(positionRots).withSlot(1));
    }
    //TODO: Add .withVelocity() to negate robot velocity

    @Override
    public void toTurretVoltage(final double volts) {
        turretMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toTurretTorqueCurrent(final double torqueCurrent) {
        turretMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrent));
    }

    @Override
    public void setTurretPosition(final double turretAbsolutePosition) {
        this.turretMotor.setPosition(turretAbsolutePosition);
    }
}
