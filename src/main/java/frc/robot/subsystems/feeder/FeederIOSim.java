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

public class FeederIOSim implements FeederIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.FeederConstants constants;

    private final TalonFX rollerMotor;
    private final TalonFXSim rollerTalonFXSim;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerDeviceTemp;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;

    public FeederIOSim(final HardwareConstants.FeederConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final DCMotorSim rollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        2 / (2 * Math.PI),
                        1 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(1)
        );

        this.rollerMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.rollerTalonFXSim = new TalonFXSim(
                rollerMotor,
                constants.rollerGearing(),
                rollerSim::update,
                rollerSim::setInputVoltage,
                rollerSim::getAngularPositionRad,
                rollerSim::getAngularVelocityRadPerSec
        );

        this.rollerPosition = rollerMotor.getPosition(false);
        this.rollerVelocity = rollerMotor.getVelocity(false);
        this.rollerVoltage = rollerMotor.getMotorVoltage(false);
        this.rollerTorqueCurrent = rollerMotor.getTorqueCurrent(false);
        this.rollerDeviceTemp = rollerMotor.getDeviceTemp(false);

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

        RefreshAll.add(
                constants.CANBus(),
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                rollerDeviceTemp
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            rollerTalonFXSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d)",
                rollerMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        final TalonFXConfiguration rollerConfiguration = new TalonFXConfiguration();
        rollerConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.01)
                .withKP(3.5);
        rollerConfiguration.CurrentLimits.StatorCurrentLimit = 50;
        rollerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        rollerConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        rollerConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rollerConfiguration.Feedback.SensorToMechanismRatio = constants.rollerGearing();
        rollerMotor.getConfigurator().apply(rollerConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                rollerDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                rollerMotor
        );

        rollerMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    }

    @Override
    public void updateInputs(FeederIO.FeederIOInputs inputs) {
        inputs.rollerPositionRots = rollerPosition.getValueAsDouble();
        inputs.rollerVelocityRotsPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toRollerVelocity(final double velocityRotsPerSec) {
        rollerMotor.setControl(velocityTorqueCurrentFOC.withVelocity(velocityRotsPerSec));
    }
}
