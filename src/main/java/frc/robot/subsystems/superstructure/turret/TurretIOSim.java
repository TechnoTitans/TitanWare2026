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
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.utils.sim.feedback.SimFeedbackSensor;
import frc.robot.utils.sim.motors.TalonFXSim;

public class TurretIOSim implements TurretIO {
    private final DeltaTime deltaTime;
    private final HardwareConstants.TurretConstants constants;

    private final TalonFX motor;
    private final CANcoder primaryCANcoder;
    private final CANcoder secondaryCANcoder;

    private final DCMotorSim dcMotorSim;
    private final Rotation2d initialRandPosition;

    private final TalonFXSim motorSim;
    private final SimTurretCANcoders simCANcoders;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    private final StatusSignal<Angle> primaryEncoderPosition;
    private final StatusSignal<Angle> primaryEncoderAbsolutePosition;

    private final StatusSignal<Angle> secondaryEncoderPosition;
    private final StatusSignal<Angle> secondaryEncoderAbsolutePosition;

    private final PositionVoltage positionVoltage;
    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final VoltageOut voltageOut;

    public TurretIOSim(final HardwareConstants.TurretConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.motor = new TalonFX(constants.motorID(), p6Bus);
        this.primaryCANcoder = new CANcoder(constants.primaryCANcoderID(), p6Bus);
        this.secondaryCANcoder = new CANcoder(constants.secondaryCANcoderID(), p6Bus);

        this.initialRandPosition = Rotation2d.fromRotations(
                Math.random()
                        * (constants.forwardLimitRots() - constants.reverseLimitRots())
                        + constants.reverseLimitRots()
        );

        final double motorToTurretGearing = constants.motorToGearboxGearing() * constants.gearboxToTurretGearing();
        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(1);
        this.dcMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        dcMotor,
                        0.0977,
                        motorToTurretGearing
                ),
                dcMotor
        );
        this.dcMotorSim.setState(initialRandPosition.getRadians(), 0);

        this.motorSim = new TalonFXSim(
                motor,
                motorToTurretGearing,
                dcMotorSim::update,
                voltage -> dcMotorSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.25)),
                dcMotorSim::getAngularPositionRad,
                dcMotorSim::getAngularVelocityRadPerSec
        );
        this.simCANcoders = new SimTurretCANcoders(constants, primaryCANcoder, secondaryCANcoder);

        this.positionVoltage = new PositionVoltage(0);
        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.voltageOut = new VoltageOut(0);

        this.motorPosition = motor.getPosition(false);
        this.motorVelocity = motor.getVelocity(false);
        this.motorVoltage = motor.getMotorVoltage(false);
        this.motorTorqueCurrent = motor.getTorqueCurrent(false);
        this.motorDeviceTemp = motor.getDeviceTemp(false);

        this.primaryEncoderPosition = primaryCANcoder.getPosition(false);
        this.primaryEncoderAbsolutePosition = primaryCANcoder.getAbsolutePosition(false);

        this.secondaryEncoderPosition = secondaryCANcoder.getPosition(false);
        this.secondaryEncoderAbsolutePosition = secondaryCANcoder.getAbsolutePosition(false);

        RefreshAll.add(
                bus,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                motorDeviceTemp,
                primaryEncoderPosition,
                primaryEncoderAbsolutePosition,
                secondaryEncoderPosition,
                secondaryEncoderAbsolutePosition
        );

        config();

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            motorSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d)",
                motor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SimConstants.SIM_UPDATE_PERIODIC_SEC);
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.25)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(5.248)
                .withKA(0.0088)
                .withKP(103.385)
                .withKD(5.169);
        motorConfiguration.Slot1 = new Slot1Configs()
                .withKS(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKV(0)
                .withKA(0)
                .withKP(80)
                .withKD(4);
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorConfiguration.MotionMagic.MotionMagicExpo_kV = 0.12;
        motorConfiguration.MotionMagic.MotionMagicExpo_kA = 0.1;
        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
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
        primaryCANcoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        primaryCANcoderConfiguration.MagnetSensor.MagnetOffset = 0;
        Phoenix6Utils.tryUntilOk(
                primaryCANcoder,
                () -> primaryCANcoder.getConfigurator().apply(primaryCANcoderConfiguration)
        );

        final CANcoderConfiguration secondaryCANcoderConfiguration = new CANcoderConfiguration();
        secondaryCANcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        secondaryCANcoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        secondaryCANcoderConfiguration.MagnetSensor.MagnetOffset = 0;
        Phoenix6Utils.tryUntilOk(
                secondaryCANcoder,
                () -> secondaryCANcoder.getConfigurator().apply(secondaryCANcoderConfiguration)
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                primaryEncoderPosition,
                primaryEncoderAbsolutePosition,
                secondaryEncoderPosition,
                secondaryEncoderAbsolutePosition
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

        final CANcoderSimState primaryCANcoderSimState = primaryCANcoder.getSimState();
        primaryCANcoderSimState.Orientation = ChassisReference.Clockwise_Positive;
        primaryCANcoderSimState.SensorOffset = 0;

        final CANcoderSimState secondaryCANcoderSimState = secondaryCANcoder.getSimState();
        secondaryCANcoderSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        secondaryCANcoderSimState.SensorOffset = 0;

        simCANcoders.setRawPosition(initialRandPosition.getRotations());
        motorSim.attachFeedbackSensor(simCANcoders);
    }

    @Override
    public void updateInputs(final TurretIOInputs inputs) {
        inputs.turretPositionRots = motorPosition.getValueAsDouble();
        inputs.turretVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.turretVoltage = motorVoltage.getValueAsDouble();
        inputs.turretTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.turretTempCelsius = motorDeviceTemp.getValueAsDouble();

        inputs.primaryEncoderPositionRots = primaryEncoderPosition.getValueAsDouble();
        inputs.primaryEncoderAbsolutePositionRots = primaryEncoderAbsolutePosition.getValueAsDouble();

        inputs.secondaryEncoderPositionRots = secondaryEncoderPosition.getValueAsDouble();
        inputs.secondaryEncoderAbsolutePositionRots = secondaryEncoderAbsolutePosition.getValueAsDouble();
    }

    @Override
    public void seedTurretPosition(final Rotation2d turretPosition) {
        final double turretPositionRots = turretPosition.getRotations();
        Phoenix6Utils.tryUntilOk(motor, 10, () -> motor.setPosition(turretPositionRots));
    }

    @Override
    public void toTurretContinuousPosition(double positionRots, double velocityRotsPerSec) {
        motor.setControl(positionVoltage
                .withSlot(0)
                .withPosition(positionRots)
                .withVelocity(velocityRotsPerSec)
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
            Phoenix6Utils.reportIfNotOk(
                    primaryCANcoder,
                    primarySimState.setRawPosition(rotations / constants.primaryCANcoderGearing())
            );

            Phoenix6Utils.reportIfNotOk(
                    secondaryCANcoder,
                    secondarySimState.setRawPosition(rotations / constants.secondaryCANcoderGearing())
            );
        }

        @Override
        public void addPosition(final double deltaRotations) {
            Phoenix6Utils.reportIfNotOk(
                    primaryCANcoder,
                    primarySimState.addPosition(deltaRotations / constants.primaryCANcoderGearing())
            );

            Phoenix6Utils.reportIfNotOk(
                    secondaryCANcoder,
                    secondarySimState.addPosition(deltaRotations / constants.secondaryCANcoderGearing())
            );
        }

        @Override
        public void setVelocity(final double rotationsPerSec) {
            Phoenix6Utils.reportIfNotOk(
                    primaryCANcoder,
                    primarySimState.setVelocity(rotationsPerSec / constants.primaryCANcoderGearing())
            );

            Phoenix6Utils.reportIfNotOk(
                    secondaryCANcoder,
                    secondarySimState.setVelocity(rotationsPerSec / constants.secondaryCANcoderGearing())
            );
        }
    }
}