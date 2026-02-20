package frc.robot.subsystems.spindexer;

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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.motors.TalonFXSim;

public class SpindexerIOSim implements SpindexerIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.SpindexerConstants constants;

    private final TalonFX wheelMotor;
    private final TalonFXSim wheelTalonFXSim;

    private final StatusSignal<Angle> wheelPosition;
    private final StatusSignal<AngularVelocity> wheelVelocity;
    private final StatusSignal<Voltage> wheelVoltage;
    private final StatusSignal<Current> wheelTorqueCurrent;
    private final StatusSignal<Temperature> wheelTemperature;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;

    public SpindexerIOSim(final HardwareConstants.SpindexerConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final DCMotorSim wheelSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        1 / (2 * Math.PI),
                        0.1 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(1)
        );

        this.wheelMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.wheelTalonFXSim = new TalonFXSim(
                wheelMotor,
                constants.wheelGearing(),
                wheelSim::update,
                wheelSim::setInputVoltage,
                wheelSim::getAngularPositionRad,
                wheelSim::getAngularVelocityRadPerSec
        );

        this.wheelPosition = wheelMotor.getPosition(false);
        this.wheelVelocity = wheelMotor.getVelocity(false);
        this.wheelVoltage = wheelMotor.getMotorVoltage(false);
        this.wheelTorqueCurrent = wheelMotor.getTorqueCurrent(false);
        this.wheelTemperature = wheelMotor.getDeviceTemp(false);

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

        RefreshAll.add(
                constants.CANBus(),
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                wheelTemperature
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
                .withKS(0.01)
                .withKP(5);
        wheelConfiguration.CurrentLimits.StatorCurrentLimit = 40;
        wheelConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        wheelConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        wheelConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        wheelConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;
        wheelConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        wheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wheelConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
                wheelTemperature
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                wheelMotor
        );

        wheelMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.wheelPositionRots = wheelPosition.getValueAsDouble();
        inputs.wheelVelocityRotsPerSec = wheelVelocity.getValueAsDouble();
        inputs.wheelVoltage = wheelVoltage.getValueAsDouble();
        inputs.wheelTorqueCurrentAmps = wheelTorqueCurrent.getValueAsDouble();
        inputs.wheelTemperatureCelsius = wheelTemperature.getValueAsDouble();
    }

    @Override
    public void toWheelVelocity(final double wheelVelocityRotsPerSec) {
        wheelMotor.setControl(velocityTorqueCurrentFOC.withVelocity(wheelVelocityRotsPerSec));
    }
}
