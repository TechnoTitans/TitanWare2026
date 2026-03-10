package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.motors.TalonFXSim;

public class HoodIOSim implements HoodIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.HoodConstants constants;

    private final TalonFX motor;
    private final TalonFXSim motorSim;

    private final PositionVoltage positionVoltage;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    public HoodIOSim(final HardwareConstants.HoodConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.motor = new TalonFX(constants.motorId(), p6Bus);

        final DCMotor dcMotor = DCMotor.getKrakenX44Foc(1);
        final SingleJointedArmSim armSim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(dcMotor,0.04, constants.gearing()),
                dcMotor,
                constants.gearing(),
                Units.inchesToMeters(8),
//                Units.rotationsToRadians(constants.lowerLimitRots()),
                0,
//                Units.rotationsToRadians(constants.upperLimitRots()),
                Units.degreesToRadians(90),
                false,
                0
        );

        this.motorSim = new TalonFXSim(
                motor,
                constants.gearing(),
                armSim::update,
                armSim::setInputVoltage,
                armSim::getAngleRads,
                armSim::getVelocityRadPerSec
        );

        this.positionVoltage = new PositionVoltage(0);
        this.voltageOut = new VoltageOut(0);

        this.motorPosition = motor.getPosition(false);
        this.motorVelocity = motor.getVelocity(false);
        this.motorVoltage = motor.getMotorVoltage(false);
        this.motorTorqueCurrent = motor.getTorqueCurrent(false);
        this.motorDeviceTemp = motor.getDeviceTemp(false);

        RefreshAll.add(
                bus,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                motorDeviceTemp
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
    public void updateInputs(final HoodIO.HoodIOInputs inputs) {
        inputs.pivotPositionRots = motorPosition.getValueAsDouble();
        inputs.pivotVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.pivotVoltage = motorVoltage.getValueAsDouble();
        inputs.pivotTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.pivotTempCelsius = motorDeviceTemp.getValueAsDouble();
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.35)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKP(375)
                .withKD(0);
        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.gearing();
        motorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfiguration));

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                motorDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor
        );

        final TalonFXSimState motorSimState = motor.getSimState();
        motorSimState.Orientation = ChassisReference.Clockwise_Positive;
        motorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }

    @Override
    public void toHoodPosition(final double hoodPositionRots) {
        motor.setControl(positionVoltage.withPosition(hoodPositionRots));
    }

    @Override
    public void toHoodVoltage(final double hoodVolts) {
        motor.setControl(voltageOut.withOutput(hoodVolts));
    }

    @Override
    public void setPosition(final double positionRots) {
        motor.setPosition(positionRots);
    }
}
