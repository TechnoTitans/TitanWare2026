package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.motors.TalonFXSim;

public class IntakeRollerIOSim implements IntakeRollerIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.IntakeRollerConstants constants;

    private final TalonFX rollerMotor;
    private final TalonFXSim motorSim;

    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerDeviceTemp;

    public IntakeRollerIOSim(final HardwareConstants.IntakeRollerConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        this.rollerMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(1);
        final DCMotorSim dcMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(dcMotor, 0.001, constants.rollerGearing()),
                dcMotor
        );
        this.motorSim = new TalonFXSim(
                rollerMotor,
                constants.rollerGearing(),
                dcMotorSim::update,
                voltage -> dcMotorSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.25)),
                dcMotorSim::getAngularPositionRad,
                dcMotorSim::getAngularVelocityRadPerSec
        );

        this.voltageOut = new VoltageOut(0);

        this.rollerPosition = rollerMotor.getPosition(false);
        this.rollerVelocity = rollerMotor.getVelocity(false);
        this.rollerVoltage = rollerMotor.getMotorVoltage(false);
        this.rollerTorqueCurrent = rollerMotor.getTorqueCurrent(false);
        this.rollerDeviceTemp = rollerMotor.getDeviceTemp(false);

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
            motorSim.update(dt);
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
        final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLowerTime = 2.5;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = constants.rollerGearing();
        Phoenix6Utils.tryUntilOk(rollerMotor, () -> rollerMotor.getConfigurator().apply(talonFXConfiguration));

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

        final TalonFXSimState rollerSimState = rollerMotor.getSimState();
        rollerSimState.Orientation = ChassisReference.Clockwise_Positive;
        rollerSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.rollerPositionRots = rollerPosition.getValueAsDouble();
        inputs.rollerVelocityRotsPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toRollerVoltage(final double volts) {
        rollerMotor.setControl(voltageOut.withOutput(volts));
    }
}
