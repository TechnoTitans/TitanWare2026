package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
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

import java.util.List;

public class IntakeRollersIOSim implements IntakeRollersIO {
    private final DeltaTime deltaTime;
    private final HardwareConstants.IntakeRollerConstants constants;

    private final TalonFX masterMotor;
    private final TalonFX followerMotor;

    private final TalonFXSim motorsSim;

    private final StatusSignal<Angle> masterPosition;
    private final StatusSignal<AngularVelocity> masterVelocity;
    private final StatusSignal<Voltage> masterVoltage;
    private final StatusSignal<Current> masterTorqueCurrent;
    private final StatusSignal<Temperature> masterDeviceTemp;

    private final StatusSignal<Angle> followerPosition;
    private final StatusSignal<AngularVelocity> followerVelocity;
    private final StatusSignal<Voltage> followerVoltage;
    private final StatusSignal<Current> followerTorqueCurrent;
    private final StatusSignal<Temperature> followerDeviceTemp;

    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final Follower follower;

    public IntakeRollersIOSim(final HardwareConstants.IntakeRollerConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.masterMotor = new TalonFX(constants.masterMotorId(), p6Bus);
        this.followerMotor = new TalonFX(constants.followerMotorId(), p6Bus);

        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(2);
        final DCMotorSim motorsSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        dcMotor,
                        0.001,
                        constants.gearing()),
                dcMotor
        );
        this.motorsSim = new TalonFXSim(
                List.of(masterMotor, followerMotor),
                constants.gearing(),
                motorsSim::update,
                voltage -> motorsSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.25)),
                motorsSim::getAngularPositionRad,
                motorsSim::getAngularVelocityRadPerSec
        );

        this.masterPosition = masterMotor.getPosition(false);
        this.masterVelocity = masterMotor.getVelocity(false);
        this.masterVoltage = masterMotor.getMotorVoltage(false);
        this.masterTorqueCurrent = masterMotor.getTorqueCurrent(false);
        this.masterDeviceTemp = masterMotor.getDeviceTemp(false);

        this.followerPosition = followerMotor.getPosition(false);
        this.followerVelocity = followerMotor.getVelocity(false);
        this.followerVoltage = followerMotor.getMotorVoltage(false);
        this.followerTorqueCurrent = followerMotor.getTorqueCurrent(false);
        this.followerDeviceTemp = followerMotor.getDeviceTemp(false);

        this.voltageOut = new VoltageOut(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.follower = new Follower(masterMotor.getDeviceID(), MotorAlignmentValue.Opposed);

        RefreshAll.add(
                bus,
                masterPosition,
                masterVelocity,
                masterVoltage,
                masterTorqueCurrent,
                masterDeviceTemp,
                followerPosition,
                followerVelocity,
                followerVoltage,
                followerTorqueCurrent,
                followerDeviceTemp
        );

        config();

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            this.motorsSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d, %d)",
                masterMotor.getDeviceID(),
                followerMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SimConstants.SIM_UPDATE_PERIODIC_SEC);
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(3.2198)
                .withKV(0.068888)
                .withKA(0.15632)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKP(5.5)
                .withKD(0);
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
        Phoenix6Utils.tryUntilOk(masterMotor, () -> masterMotor.getConfigurator().apply(motorConfig));

        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        Phoenix6Utils.tryUntilOk(followerMotor, () -> followerMotor.getConfigurator().apply(motorConfig));


        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                masterPosition,
                masterVelocity,
                masterVoltage,
                masterTorqueCurrent,
                followerPosition,
                followerVelocity,
                followerVoltage,
                followerTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                masterDeviceTemp,
                followerDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                masterMotor,
                followerMotor
        );

        final TalonFXSimState masterMotorSimStateSimState = masterMotor.getSimState();
        masterMotorSimStateSimState.Orientation = ChassisReference.Clockwise_Positive;
        masterMotorSimStateSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        final TalonFXSimState followerMotorSimStateSimState = followerMotor.getSimState();
        followerMotorSimStateSimState.Orientation = ChassisReference.Clockwise_Positive;
        followerMotorSimStateSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.masterPositionRots = masterPosition.getValueAsDouble();
        inputs.masterVelocityRotsPerSec = masterVelocity.getValueAsDouble();
        inputs.masterVoltage = masterVoltage.getValueAsDouble();
        inputs.masterTorqueCurrentAmps = masterTorqueCurrent.getValueAsDouble();
        inputs.masterTempCelsius = masterDeviceTemp.getValueAsDouble();

        inputs.followerPositionRots = followerPosition.getValueAsDouble();
        inputs.followerVelocityRotsPerSec = followerVelocity.getValueAsDouble();
        inputs.followerVoltage = followerVoltage.getValueAsDouble();
        inputs.followerTorqueCurrentAmps = followerTorqueCurrent.getValueAsDouble();
        inputs.followerTempCelsius = followerDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toRollersVoltage(final double volts) {
        masterMotor.setControl(voltageOut.withOutput(volts));
        followerMotor.setControl(follower);
    }

    @Override
    public void toRollersTorqueCurrent(final double torqueCurrentAmps) {
        masterMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
        followerMotor.setControl(follower);
    }

    @Override
    public void toRollersVelocity(final double velocityRotsPerSec) {
        masterMotor.setControl(velocityTorqueCurrentFOC.withVelocity(velocityRotsPerSec));
        followerMotor.setControl(follower);
    }
}
