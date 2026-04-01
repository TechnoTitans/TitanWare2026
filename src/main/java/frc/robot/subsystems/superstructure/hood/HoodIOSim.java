package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import frc.robot.constants.SimConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.motors.TalonFXSim;

public class HoodIOSim implements HoodIO {
    private final DeltaTime deltaTime;
    private final HardwareConstants.HoodConstants constants;

    private final TalonFX motor;
    private final TalonFXSim motorSim;

    private final PositionVoltage positionVoltage;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    public HoodIOSim(final HardwareConstants.HoodConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final HardwareConstants.CANBus canBus = constants.CANBus();
        this.motor = new TalonFX(constants.motorId(), canBus.toPhoenix6CANBus());

        final DCMotor dcMotor = DCMotor.getKrakenX44Foc(1);
        final SingleJointedArmSim armSim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        dcMotor,
                        SimConstants.Hood.MOMENT_OF_INERTIA,
                        constants.gearing()
                ),
                dcMotor,
                constants.gearing(),
                Units.inchesToMeters(8),
                0,
                Units.degreesToRadians(90),
                false,
                0
        );

        this.motorSim = new TalonFXSim(
                motor,
                constants.gearing(),
                armSim::update,
                voltage -> armSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.25)),
                armSim::getAngleRads,
                armSim::getVelocityRadPerSec
        );

        this.positionVoltage = new PositionVoltage(0);

        this.motorPosition = motor.getPosition(false);
        this.motorVelocity = motor.getVelocity(false);
        this.motorVoltage = motor.getMotorVoltage(false);
        this.motorTorqueCurrent = motor.getTorqueCurrent(false);
        this.motorDeviceTemp = motor.getDeviceTemp(false);

        RefreshAll.add(
                canBus,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                motorDeviceTemp
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
    public void updateInputs(final HoodIO.HoodIOInputs inputs) {
        inputs.hoodPositionRots = motorPosition.getValueAsDouble();
        inputs.hoodVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.hoodVoltage = motorVoltage.getValueAsDouble();
        inputs.hoodTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.hoodTempCelsius = motorDeviceTemp.getValueAsDouble();
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(0.3)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKP(300)
                .withKD(0.1);
        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 45;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.gearing();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfig));

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
    public void zero() {
        Phoenix6Utils.tryUntilOk(motor, () -> motor.setPosition(0));
    }
}