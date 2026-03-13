package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.motors.TalonFXSim;

public class FeederIOSim implements FeederIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.FeederConstants constants;

    private final TalonFX wheelMotor;
    private final TalonFXSim wheelTalonFXSim;

    private final StatusSignal<Angle> wheelPosition;
    private final StatusSignal<AngularVelocity> wheelVelocity;
    private final StatusSignal<Voltage> wheelVoltage;
    private final StatusSignal<Current> wheelTorqueCurrent;
    private final StatusSignal<Temperature> wheelDeviceTemp;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;

    public FeederIOSim(final HardwareConstants.FeederConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(1);
        final DCMotorSim dcMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(dcMotor, 0.0026, constants.wheelGearing()),
                dcMotor
        );

        this.wheelMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.wheelTalonFXSim = new TalonFXSim(
                wheelMotor,
                constants.wheelGearing(),
                dcMotorSim::update,
                voltage -> dcMotorSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.25)),
                dcMotorSim::getAngularPositionRad,
                dcMotorSim::getAngularVelocityRadPerSec
        );

        this.wheelPosition = wheelMotor.getPosition(false);
        this.wheelVelocity = wheelMotor.getVelocity(false);
        this.wheelVoltage = wheelMotor.getMotorVoltage(false);
        this.wheelTorqueCurrent = wheelMotor.getTorqueCurrent(false);
        this.wheelDeviceTemp = wheelMotor.getDeviceTemp(false);

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

        RefreshAll.add(
                constants.CANBus(),
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                wheelDeviceTemp
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            wheelTalonFXSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d)",
                wheelMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        final TalonFXConfiguration wheelConfiguration = new TalonFXConfiguration();
        wheelConfiguration.Slot0 = new Slot0Configs()
                .withKS(6.72)
                .withKV(0.08)
                .withKP(6)
                .withKD(0.09);
        wheelConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        wheelConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        wheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wheelConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        wheelConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        wheelConfiguration.Feedback.SensorToMechanismRatio = constants.wheelGearing();
        wheelMotor.getConfigurator().apply(wheelConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                wheelDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                wheelMotor
        );

        wheelMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        wheelMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void updateInputs(FeederIO.FeederIOInputs inputs) {
        inputs.wheelPositionRots = wheelPosition.getValueAsDouble();
        inputs.wheelVelocityRotsPerSec = wheelVelocity.getValueAsDouble();
        inputs.wheelVoltage = wheelVoltage.getValueAsDouble();
        inputs.wheelTorqueCurrentAmps = wheelTorqueCurrent.getValueAsDouble();
        inputs.wheelTempCelsius = wheelDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toWheelVelocity(final double velocityRotsPerSec) {
        wheelMotor.setControl(velocityTorqueCurrentFOC.withVelocity(velocityRotsPerSec));
    }
}
