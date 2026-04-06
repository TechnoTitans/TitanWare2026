package frc.robot.subsystems.indexer.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.CANrangeSimState;
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
import frc.robot.utils.sim.motors.TalonFXSim;

public class FeederIOSim implements FeederIO {
    private static final double CloseEnoughMeters = 0.01;

    private final DeltaTime deltaTime;
    private final HardwareConstants.FeederConstants constants;

    private final TalonFX motor;
    private final TalonFXSim motorTalonFXSim;

    private final CANrange canRange;
    private final CANrangeConfiguration canRangeConfiguration;
    private final CANrangeSimState canRangeSimState;

    private final StatusSignal<Angle> wheelPosition;
    private final StatusSignal<AngularVelocity> wheelVelocity;
    private final StatusSignal<Voltage> wheelVoltage;
    private final StatusSignal<Current> wheelTorqueCurrent;
    private final StatusSignal<Temperature> wheelDeviceTemp;

    private final StatusSignal<Boolean> canRangeDetected;

    private final TorqueCurrentFOC torqueCurrentFOC;

    public FeederIOSim(final HardwareConstants.FeederConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.motor = new TalonFX(constants.motorId(), p6Bus);
        this.canRange = new CANrange(constants.CANRangeId(), p6Bus);

        this.canRangeConfiguration = new CANrangeConfiguration();
        this.canRangeSimState = canRange.getSimState();

        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(1);
        final DCMotorSim dcMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(dcMotor, SimConstants.Feeder.MOMENT_OF_INERTIA, constants.gearing()),
                dcMotor
        );

        this.motorTalonFXSim = new TalonFXSim(
                motor,
                constants.gearing(),
                dcMotorSim::update,
                voltage -> dcMotorSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.25)),
                dcMotorSim::getAngularPositionRad,
                dcMotorSim::getAngularVelocityRadPerSec
        );

        this.wheelPosition = motor.getPosition(false);
        this.wheelVelocity = motor.getVelocity(false);
        this.wheelVoltage = motor.getMotorVoltage(false);
        this.wheelTorqueCurrent = motor.getTorqueCurrent(false);
        this.wheelDeviceTemp = motor.getDeviceTemp(false);

        this.canRangeDetected = canRange.getIsDetected(false);

        this.torqueCurrentFOC = new TorqueCurrentFOC(0);

        RefreshAll.add(
                bus,
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                wheelDeviceTemp,
                canRangeDetected
        );

        config();

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            motorTalonFXSim.update(dt);
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
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(14)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKV(0)
                .withKP(10)
                .withKD(0);
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        motorConfig.CurrentLimits.StatorCurrentLimit = 80;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 80;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 55;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.gearing();
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfig));

        canRangeConfiguration.ProximityParams.ProximityThreshold = 0.05;
        canRangeConfiguration.ProximityParams.ProximityHysteresis = 0.02;
        canRangeConfiguration.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
        canRangeConfiguration.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        Phoenix6Utils.tryUntilOk(canRange, () -> canRange.getConfigurator().apply(canRangeConfiguration));

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                canRangeDetected
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                wheelDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor,
                canRange
        );

        final TalonFXSimState wheelMotorSimState = motor.getSimState();
        wheelMotorSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        wheelMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void updateInputs(FeederIO.FeederIOInputs inputs) {
        inputs.wheelPositionRots = wheelPosition.getValueAsDouble();
        inputs.wheelVelocityRotsPerSec = wheelVelocity.getValueAsDouble();
        inputs.wheelVoltage = wheelVoltage.getValueAsDouble();
        inputs.wheelTorqueCurrentAmps = wheelTorqueCurrent.getValueAsDouble();
        inputs.wheelTempCelsius = wheelDeviceTemp.getValueAsDouble();

        inputs.tofDetected = canRangeDetected.getValue();
    }

    @Override
    public void toWheelTorqueCurrent(final double torqueCurrentAmps) {
        motor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }

    @Override
    public void setTOFDetected(final boolean isDetected) {
        final double threshold = canRangeConfiguration.ProximityParams.ProximityThreshold;
        final double hysteresis = canRangeConfiguration.ProximityParams.ProximityHysteresis;

        canRangeSimState.setDistance(isDetected
                ? threshold - hysteresis - CloseEnoughMeters
                : threshold + hysteresis + CloseEnoughMeters
        );
    }
}
