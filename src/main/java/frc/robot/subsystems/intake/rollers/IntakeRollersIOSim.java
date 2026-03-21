package frc.robot.subsystems.intake.rollers;

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
import frc.robot.constants.SimConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.motors.TalonFXSim;

public class IntakeRollersIOSim implements IntakeRollersIO {
    private final DeltaTime deltaTime;
    private final HardwareConstants.IntakeRollerConstants constants;

    private final TalonFX motor;
    private final TalonFXSim motorSim;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerDeviceTemp;

    private final VoltageOut voltageOut;

    public IntakeRollersIOSim(final HardwareConstants.IntakeRollerConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        this.motor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(1);
        final DCMotorSim dcMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(dcMotor, 0.001, constants.gearing()),
                dcMotor
        );
        this.motorSim = new TalonFXSim(
                motor,
                constants.gearing(),
                dcMotorSim::update,
                voltage -> dcMotorSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.25)),
                dcMotorSim::getAngularPositionRad,
                dcMotorSim::getAngularVelocityRadPerSec
        );

        this.rollerPosition = motor.getPosition(false);
        this.rollerVelocity = motor.getVelocity(false);
        this.rollerVoltage = motor.getMotorVoltage(false);
        this.rollerTorqueCurrent = motor.getTorqueCurrent(false);
        this.rollerDeviceTemp = motor.getDeviceTemp(false);

        this.voltageOut = new VoltageOut(0);

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
                motor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SimConstants.SIM_UPDATE_PERIODIC_SEC);
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 2.5;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.gearing();
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfig));

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
                motor
        );

        final TalonFXSimState rollerSimState = motor.getSimState();
        rollerSimState.Orientation = ChassisReference.Clockwise_Positive;
        rollerSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.rollersPositionRots = rollerPosition.getValueAsDouble();
        inputs.rollersVelocityRotsPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollersVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollersTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollersTempCelsius = rollerDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toRollersVoltage(final double volts) {
        motor.setControl(voltageOut.withOutput(volts));
    }
}
