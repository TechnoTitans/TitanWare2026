package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
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

public class ShooterIOSim implements ShooterIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.ShooterConstants constants;

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

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final Follower follower;

    public ShooterIOSim(final HardwareConstants.ShooterConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        this.masterMotor = new TalonFX(constants.masterMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.followerMotor = new TalonFX(constants.followerMotorID(), constants.CANBus().toPhoenix6CANBus());

        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(2);
        final DCMotorSim motorsSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        dcMotor,
                        SimConstants.Shooter.MOMENT_OF_INERTIA,
                        constants.gearing()
                ),
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

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.follower = new Follower(masterMotor.getDeviceID(), MotorAlignmentValue.Aligned);

        RefreshAll.add(
                constants.CANBus(),
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

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            this.motorsSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d,%d)",
                masterMotor.getDeviceID(),
                followerMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(6.75)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(0.13)
                .withKA(0.21318)
                .withKP(11)
                .withKD(0);
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        motorConfig.CurrentLimits.StatorCurrentLimit = 80;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 75;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 60;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 2.5;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.gearing();
        Phoenix6Utils.tryUntilOk(masterMotor, () -> masterMotor.getConfigurator().apply(motorConfig));

        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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

        final TalonFXSimState masterMotorSimState = masterMotor.getSimState();
        masterMotorSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        masterMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        final TalonFXSimState followerMotorSimState = followerMotor.getSimState();
        followerMotorSimState.Orientation = ChassisReference.Clockwise_Positive;
        followerMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
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
    public void toFlywheelVelocity(final double velocityRotsPerSec) {
        masterMotor.setControl(velocityTorqueCurrentFOC.withVelocity(velocityRotsPerSec));
        followerMotor.setControl(follower);
    }

    @Override
    public void toFlywheelVoltage(final double volts) {
        masterMotor.setControl(voltageOut.withOutput(volts));
        followerMotor.setControl(follower);
    }

    @Override
    public void toFlywheelTorqueCurrent(final double torqueCurrentAmps) {
        masterMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
        followerMotor.setControl(follower);
    }
}
