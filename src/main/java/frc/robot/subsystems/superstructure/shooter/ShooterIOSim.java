package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final Follower follower;

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

    public ShooterIOSim(final HardwareConstants.ShooterConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.masterMotor = new TalonFX(constants.masterId(), p6Bus);
        this.followerMotor = new TalonFX(constants.followerId(), p6Bus);

        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(2);
        final DCMotorSim dcMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(dcMotor, 0.01, constants.gearing()),
                dcMotor
        );
        this.motorsSim = new TalonFXSim(
                List.of(masterMotor, followerMotor),
                constants.gearing(),
                dcMotorSim::update,
                voltage -> dcMotorSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.25)),
                dcMotorSim::getAngularPositionRad,
                dcMotorSim::getAngularVelocityRadPerSec
        );

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);
        this.follower = new Follower(masterMotor.getDeviceID(), MotorAlignmentValue.Opposed);

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

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            motorsSim.update(dt);
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
    public void updateInputs(final ShooterIO.ShooterIOInputs inputs) {
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
    public void config() {
        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKS(7.8)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKP(40)
                .withKD(0);
        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.gearing();
        motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        Phoenix6Utils.tryUntilOk(masterMotor, () -> masterMotor.getConfigurator().apply(motorConfiguration));
        Phoenix6Utils.tryUntilOk(followerMotor, () -> followerMotor.getConfigurator().apply(motorConfiguration));

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
    public void toShooterVelocity(final double shooterVelocityRotsPerSec) {
        masterMotor.setControl(velocityTorqueCurrentFOC.withVelocity(shooterVelocityRotsPerSec));
        followerMotor.setControl(follower);
    }

    @Override
    public void toShooterVoltage(final double shooterVolts) {
        masterMotor.setControl(voltageOut.withOutput(shooterVolts));
        followerMotor.setControl(follower);
    }
}
