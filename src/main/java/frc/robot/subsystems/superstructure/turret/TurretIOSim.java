package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.feedback.SimFeedbackSensor;
import frc.robot.utils.sim.motors.TalonFXSim;

public class TurretIOSim implements TurretIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.TurretConstants constants;

    private final TalonFX motor;
    private final CANcoder primaryCANcoder;
    private final CANcoder secondaryCANcoder;

    private final TalonFXSim motorSim;

    private final PositionVoltage positionVoltage;
    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    private final StatusSignal<Angle> primaryCANcoderPosition;
    private final StatusSignal<Angle> secondaryCANcoderPosition;

    public TurretIOSim(final HardwareConstants.TurretConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.motor = new TalonFX(constants.motorId(), p6Bus);
        this.primaryCANcoder = new CANcoder(constants.primaryCANcoderId(), p6Bus);
        this.secondaryCANcoder = new CANcoder(constants.secondaryCANcoderId(), p6Bus);

        final double motorToTurretGearing = constants.motorToGearboxGearing() * constants.gearboxToTurretGearing();
        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(1);
        final DCMotorSim dcMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        dcMotor,
                        0.0977,
                        motorToTurretGearing
                ),
                dcMotor
        );
        this.motorSim = new TalonFXSim(
                motor,
                motorToTurretGearing,
                dcMotorSim::update,
                voltage -> dcMotorSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.25)),
                dcMotorSim::getAngularPositionRad,
                dcMotorSim::getAngularVelocityRadPerSec
        );
        this.motorSim.attachFeedbackSensor(new SimTurretCANcoders(constants, primaryCANcoder, secondaryCANcoder));

        this.positionVoltage = new PositionVoltage(0);
        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.voltageOut = new VoltageOut(0);

        this.motorPosition = motor.getPosition(false);
        this.motorVelocity = motor.getVelocity(false);
        this.motorVoltage = motor.getMotorVoltage(false);
        this.motorTorqueCurrent = motor.getTorqueCurrent(false);
        this.motorDeviceTemp = motor.getDeviceTemp(false);

        this.primaryCANcoderPosition = primaryCANcoder.getPosition(false);
        this.secondaryCANcoderPosition = secondaryCANcoder.getPosition(false);

        RefreshAll.add(
                bus,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                motorDeviceTemp,
                primaryCANcoderPosition,
                secondaryCANcoderPosition
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            motorSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d)",
                motor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void updateInputs(final TurretIOInputs inputs) {
        inputs.motorPositionRots = motorPosition.getValueAsDouble();
        inputs.motorVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.motorVoltage = motorVoltage.getValueAsDouble();
        inputs.motorTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.motorTempCelsius = motorDeviceTemp.getValueAsDouble();

        inputs.primaryCANcoderPositionRots = primaryCANcoderPosition.getValueAsDouble();
        inputs.secondaryCANcoderPositionRots = secondaryCANcoderPosition.getValueAsDouble();
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.366)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(5)
                .withKP(30)
                .withKD(0);
        motorConfiguration.Slot1 = new Slot1Configs()
                .withKS(0.366)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKP(80)
                .withKD(0.45);
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorConfiguration.MotionMagic.MotionMagicExpo_kV = 0.12;
        motorConfiguration.MotionMagic.MotionMagicExpo_kA = 0.1;
        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.Feedback.RotorToSensorRatio = 1;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.motorToGearboxGearing()
                * constants.gearboxToTurretGearing();
        motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.reverseLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfiguration));

        final CANcoderConfiguration primaryCANcoderConfiguration = new CANcoderConfiguration();
        primaryCANcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        primaryCANcoderConfiguration.MagnetSensor.MagnetOffset = constants.primaryCANcoderOffsetRots();
        primaryCANcoder.getConfigurator().apply(primaryCANcoderConfiguration);

        final CANcoderConfiguration secondaryCANcoderConfiguration = new CANcoderConfiguration();
        secondaryCANcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        secondaryCANcoderConfiguration.MagnetSensor.MagnetOffset = constants.secondaryCANcoderOffsetRots();
        secondaryCANcoder.getConfigurator().apply(secondaryCANcoderConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                primaryCANcoderPosition,
                secondaryCANcoderPosition
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                motorDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor,
                primaryCANcoder,
                secondaryCANcoder
        );

        final TalonFXSimState motorSimState = motor.getSimState();
        motorSimState.Orientation = ChassisReference.Clockwise_Positive;
        motorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        primaryCANcoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        secondaryCANcoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public void seedTurretPosition(final Rotation2d turretPosition) {
        final double turretPositionRots = turretPosition.getRotations();
        final double primaryGearing = constants.primaryCANcoderGearing();
        final double primaryAbsolutePosition = primaryCANcoder.getAbsolutePosition().getValueAsDouble() * primaryGearing;

        if (!MathUtil.isNear(primaryAbsolutePosition, turretPositionRots, 1e-6, 0, 1)) {
            DriverStation.reportError(String.format(
                    "Failed to seed turret position! Expected integer increment in position from: %.3f to %.3f",
                    Math.min(primaryAbsolutePosition, turretPositionRots),
                    Math.max(primaryAbsolutePosition, turretPositionRots)
            ), true);
            return;
        }

        // TODO: needs to try until OK, maybe needs more timeout
        Phoenix6Utils.reportIfNotOk(motor, primaryCANcoder.setPosition(turretPositionRots / primaryGearing));
    }

    @Override
    public void trackTurretPosition(final double turretPositionRots, final double turretVelocityRotsPerSec) {
        motor.setControl(positionVoltage
                .withSlot(0)
                .withPosition(turretPositionRots)
                .withVelocity(turretVelocityRotsPerSec)
        );
    }

    @Override
    public void toTurretPosition(final double turretPositionRots) {
        motor.setControl(motionMagicExpoVoltage
                .withSlot(1)
                .withPosition(turretPositionRots));
    }

    @Override
    public void toTurretVoltage(final double turretVolts) {
        motor.setControl(voltageOut.withOutput(turretVolts));
    }

    private static class SimTurretCANcoders implements SimFeedbackSensor {
        private final HardwareConstants.TurretConstants constants;

        private final CANcoder primaryCANcoder;
        private final CANcoder secondaryCANcoder;

        private final CANcoderSimState primarySimState;
        private final CANcoderSimState secondarySimState;

        public SimTurretCANcoders(
                final HardwareConstants.TurretConstants constants,
                final CANcoder primaryCANcoder,
                final CANcoder secondaryCANcoder
        ) {
            this.constants = constants;
            this.primaryCANcoder = primaryCANcoder;
            this.secondaryCANcoder = secondaryCANcoder;

            this.primarySimState = primaryCANcoder.getSimState();
            this.secondarySimState = secondaryCANcoder.getSimState();
        }

        @Override
        public void setSupplyVoltage(final double volts) {
            Phoenix6Utils.reportIfNotOk(primaryCANcoder, primarySimState.setSupplyVoltage(volts));
            Phoenix6Utils.reportIfNotOk(secondaryCANcoder, secondarySimState.setSupplyVoltage(volts));
        }

        @Override
        public void setRawPosition(final double rotations) {
            Phoenix6Utils.reportIfNotOk(primaryCANcoder,
                    primarySimState.setRawPosition(rotations / constants.primaryCANcoderGearing()));

            Phoenix6Utils.reportIfNotOk(secondaryCANcoder,
                    secondarySimState.setRawPosition(rotations / constants.secondaryCANcoderGearing()));
        }

        @Override
        public void addPosition(final double deltaRotations) {
            Phoenix6Utils.reportIfNotOk(primaryCANcoder,
                    primarySimState.addPosition(deltaRotations / constants.primaryCANcoderGearing()));

            Phoenix6Utils.reportIfNotOk(secondaryCANcoder,
                    secondarySimState.addPosition(deltaRotations / constants.secondaryCANcoderGearing()));
        }

        @Override
        public void setVelocity(final double rotationsPerSec) {
            Phoenix6Utils.reportIfNotOk(primaryCANcoder,
                    primarySimState.setVelocity(rotationsPerSec / constants.primaryCANcoderGearing()));

            Phoenix6Utils.reportIfNotOk(secondaryCANcoder,
                    secondarySimState.setVelocity(rotationsPerSec / constants.secondaryCANcoderGearing()));
        }
    }
}
