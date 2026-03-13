package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
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
import frc.robot.utils.sim.feedback.SimFeedbackSensor;
import frc.robot.utils.sim.motors.TalonFXSim;

public class TurretIOSim implements TurretIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.TurretConstants constants;

    private final TalonFX turretMotor;
    private final CANcoder primaryEncoder;
    private final CANcoder secondaryEncoder;

    private final TalonFXSim turretTalonFXSim;

    private final StatusSignal<Angle> turretPosition;
    private final StatusSignal<AngularVelocity> turretVelocity;
    private final StatusSignal<Voltage> turretVoltage;
    private final StatusSignal<Current> turretTorqueCurrent;
    private final StatusSignal<Temperature> turretDeviceTemp;

    private final StatusSignal<Angle> primaryEncoderPosition;
    private final StatusSignal<Angle> secondaryEncoderPosition;

    private final PositionVoltage positionVoltage;

    public TurretIOSim(final HardwareConstants.TurretConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        this.turretMotor = new TalonFX(constants.turretMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.primaryEncoder = new CANcoder(constants.primaryEncoderID(), constants.CANBus().toPhoenix6CANBus());
        this.secondaryEncoder = new CANcoder(constants.secondaryEncoderID(), constants.CANBus().toPhoenix6CANBus());

        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(1);
        final DCMotorSim turretSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        dcMotor,
                        SimConstants.Turret.MOMENT_OF_INERTIA,
                        constants.motorToTurretGearing()
                ),
                dcMotor
        );

        this.turretTalonFXSim = new TalonFXSim(
                turretMotor,
                constants.motorToTurretGearing(),
                turretSim::update,
                voltage -> turretSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.25)),
                turretSim::getAngularPositionRad,
                turretSim::getAngularVelocityRadPerSec
        );
        this.turretTalonFXSim.attachFeedbackSensor(new SimTurretCANcoders(constants, primaryEncoder, secondaryEncoder));

        this.turretPosition = turretMotor.getPosition(false);
        this.turretVelocity = turretMotor.getVelocity(false);
        this.turretVoltage = turretMotor.getMotorVoltage(false);
        this.turretTorqueCurrent = turretMotor.getTorqueCurrent(false);
        this.turretDeviceTemp = turretMotor.getDeviceTemp(false);

        this.primaryEncoderPosition = primaryEncoder.getPosition(true);
        this.secondaryEncoderPosition = secondaryEncoder.getPosition(true);

        this.positionVoltage = new PositionVoltage(0);

        RefreshAll.add(
                constants.CANBus(),
                turretPosition,
                turretVelocity,
                turretVoltage,
                turretTorqueCurrent,
                turretDeviceTemp
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            turretTalonFXSim.update(dt);
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
                .withKS(1.9)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKP(200)
                .withKD(10);
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.motorToTurretGearing();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.reverseLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretMotor.getConfigurator().apply(motorConfig);

        final CANcoderConfiguration primaryEncoderConfig = new CANcoderConfiguration();
        primaryEncoderConfig.MagnetSensor.MagnetOffset = constants.primaryEncoderOffset();
        primaryEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        primaryEncoder.getConfigurator().apply(primaryEncoderConfig);

        final CANcoderConfiguration secondaryEncoderConfig = new CANcoderConfiguration();
        secondaryEncoderConfig.MagnetSensor.MagnetOffset = constants.secondaryEncoderOffset();
        secondaryEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        secondaryEncoder.getConfigurator().apply(secondaryEncoderConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                turretPosition,
                turretVelocity,
                turretVoltage,
                turretTorqueCurrent,
                primaryEncoderPosition,
                secondaryEncoderPosition
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                turretDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                4,
                turretMotor,
                primaryEncoder,
                secondaryEncoder
        );

        final TalonFXSimState motorSimState = turretMotor.getSimState();
        motorSimState.Orientation = ChassisReference.Clockwise_Positive;
        motorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        primaryEncoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        secondaryEncoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public void updateInputs(final TurretIOInputs inputs) {
        inputs.turretPositionRots = turretPosition.getValueAsDouble();
        inputs.turretVelocityRotsPerSec = turretVelocity.getValueAsDouble();
        inputs.turretVoltage = turretVoltage.getValueAsDouble();
        inputs.turretTorqueCurrentAmps = turretTorqueCurrent.getValueAsDouble();
        inputs.turretTempCelsius = turretDeviceTemp.getValueAsDouble();

        inputs.primaryEncoderPositionRots = primaryEncoderPosition.getValueAsDouble();
        inputs.secondaryEncoderPositionRots = secondaryEncoderPosition.getValueAsDouble();
    }

    @Override
    public void toTurretContinuousPosition(final double positionRots, final double velocityRotsPerSec) {
        turretMotor.setControl(
                positionVoltage.withPosition(positionRots)
                        .withVelocity(velocityRotsPerSec)
                        .withSlot(0)
        );
    }

    @Override
    public void setPosition(final double turretPositionRots) {
        Phoenix6Utils.reportIfNotOk(turretMotor, turretMotor.setPosition(turretPositionRots));
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
                    primarySimState.setRawPosition(rotations / constants.primaryEncoderTooth()));

            Phoenix6Utils.reportIfNotOk(secondaryCANcoder,
                    secondarySimState.setRawPosition(rotations / constants.secondaryEncoderTooth()));
        }

        @Override
        public void addPosition(final double deltaRotations) {
            Phoenix6Utils.reportIfNotOk(primaryCANcoder,
                    primarySimState.addPosition(deltaRotations / constants.primaryEncoderTooth()));

            Phoenix6Utils.reportIfNotOk(secondaryCANcoder,
                    secondarySimState.addPosition(deltaRotations / constants.secondaryEncoderTooth()));
        }

        @Override
        public void setVelocity(final double rotationsPerSec) {
            Phoenix6Utils.reportIfNotOk(primaryCANcoder,
                    primarySimState.setVelocity(rotationsPerSec / constants.primaryEncoderTooth()));

            Phoenix6Utils.reportIfNotOk(secondaryCANcoder,
                    secondarySimState.setVelocity(rotationsPerSec / constants.secondaryEncoderTooth()));
        }
    }
}
