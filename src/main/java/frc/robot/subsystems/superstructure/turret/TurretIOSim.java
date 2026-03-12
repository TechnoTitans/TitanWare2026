package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
    private final CANcoder largeEncoder;
    private final CANcoder smallEncoder;

    private final TalonFXSim turretTalonFXSim;

    private final StatusSignal<Angle> turretPosition;
    private final StatusSignal<AngularVelocity> turretVelocity;
    private final StatusSignal<Voltage> turretVoltage;
    private final StatusSignal<Current> turretTorqueCurrent;
    private final StatusSignal<Temperature> turretDeviceTemp;

    private final StatusSignal<Angle> largeEncoderPosition;
    private final StatusSignal<Angle> smallEncoderPosition;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final PositionVoltage positionVoltage;

    public TurretIOSim(final HardwareConstants.TurretConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        this.turretMotor = new TalonFX(constants.turretMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.largeEncoder = new CANcoder(constants.secondaryEncoderID(), constants.CANBus().toPhoenix6CANBus());
        this.smallEncoder = new CANcoder(constants.primaryEncoderID(), constants.CANBus().toPhoenix6CANBus());

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
        this.turretTalonFXSim.attachFeedbackSensor(new SimTurretCANcoders(constants, smallEncoder, largeEncoder));

        this.turretPosition = turretMotor.getPosition(false);
        this.turretVelocity = turretMotor.getVelocity(false);
        this.turretVoltage = turretMotor.getMotorVoltage(false);
        this.turretTorqueCurrent = turretMotor.getTorqueCurrent(false);
        this.turretDeviceTemp = turretMotor.getDeviceTemp(false);

        this.largeEncoderPosition = largeEncoder.getPosition(true);
        this.smallEncoderPosition = smallEncoder.getPosition(true);

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.positionVoltage = new PositionVoltage(0);

        RefreshAll.add(
                constants.CANBus(),
                turretPosition,
                turretVelocity,
                turretVoltage,
                turretTorqueCurrent,
                turretDeviceTemp,
                smallEncoderPosition,
                largeEncoderPosition
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
                .withKS(0.366)
                .withKV(5)
                .withKP(30)
                .withKD(0);
        motorConfig.Slot1 = new Slot1Configs()
                .withKS(0.366)
                .withKP(80)
                .withKD(0.45);
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorConfig.CurrentLimits.StatorCurrentLimit = 70;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.motorToTurretGearing();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.reverseLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretMotor.getConfigurator().apply(motorConfig);

        final CANcoderConfiguration smallEncoderConfig = new CANcoderConfiguration();
        smallEncoderConfig.MagnetSensor.MagnetOffset = constants.primaryEncoderOffset();
        smallEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        smallEncoder.getConfigurator().apply(smallEncoderConfig);

        final CANcoderConfiguration largeEncoderConfig = new CANcoderConfiguration();
        largeEncoderConfig.MagnetSensor.MagnetOffset = constants.secondaryEncoderOffset();
        largeEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        largeEncoder.getConfigurator().apply(largeEncoderConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                turretPosition,
                turretVelocity,
                turretVoltage,
                turretTorqueCurrent,
                smallEncoderPosition,
                largeEncoderPosition
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                turretDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                4,
                turretMotor,
                smallEncoder,
                largeEncoder
        );

        final TalonFXSimState motorSimState = turretMotor.getSimState();
        motorSimState.Orientation = ChassisReference.Clockwise_Positive;
        motorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        smallEncoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        largeEncoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public void updateInputs(final TurretIOInputs inputs) {
        inputs.turretPositionRots = turretPosition.getValueAsDouble();
        inputs.turretVelocityRotsPerSec = turretVelocity.getValueAsDouble();
        inputs.turretVoltage = turretVoltage.getValueAsDouble();
        inputs.turretTorqueCurrentAmps = turretTorqueCurrent.getValueAsDouble();
        inputs.turretTempCelsius = turretDeviceTemp.getValueAsDouble();

        inputs.primaryEncoderPositionRots = smallEncoderPosition.getValueAsDouble();
        inputs.secondaryEncoderPositionRots = largeEncoderPosition.getValueAsDouble();
    }

    @Override
    public void toTurretContinuousPosition(final double positionRots, final double velocityRotsPerSec) {
        turretMotor.setControl(
                positionVoltage.withPosition(positionRots).withVelocity(-velocityRotsPerSec).withSlot(0)
        );
    }

    @Override
    public void toTurretPosition(final double positionRots) {
        turretMotor.setControl(motionMagicExpoVoltage.withPosition(positionRots).withSlot(1));
    }

    @Override
    public void seedTurretPosition(final Rotation2d turretPosition) {
        final double turretPositionRots = turretPosition.getRotations();
        final double primaryGearing = constants.primaryEncoderTooth();
        final double primaryAbsolutePosition = smallEncoder.getAbsolutePosition().getValueAsDouble() * primaryGearing;

        if (!MathUtil.isNear(primaryAbsolutePosition, turretPositionRots, 1e-6, 0, 1)) {
            DriverStation.reportError(String.format(
                    "Failed to seed turret position! Expected integer increment in position from: %.3f to %.3f",
                    Math.min(primaryAbsolutePosition, turretPositionRots),
                    Math.max(primaryAbsolutePosition, turretPositionRots)
            ), true);
            return;
        }

        Phoenix6Utils.reportIfNotOk(turretMotor, smallEncoder.setPosition(turretPositionRots / primaryGearing));
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
