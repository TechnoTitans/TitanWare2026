package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

    private final TalonFX hoodMotor;
    private final TalonFXSim hoodTalonFXSim;
    private final TalonFXConfiguration motorConfig;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodVoltage;
    private final StatusSignal<Current> hoodTorqueCurrent;
    private final StatusSignal<Temperature> hoodDeviceTemp;

    private final PositionVoltage positionVoltage;

    public HoodIOSim(final HardwareConstants.HoodConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final DCMotor dcMotor = DCMotor.getKrakenX44Foc(1);
        final SingleJointedArmSim armSim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(dcMotor,0.04, constants.gearing()),
                dcMotor,
                constants.gearing(),
                Units.inchesToMeters(8),
                Units.rotationsToRadians(constants.lowerLimitRots()),
                Units.rotationsToRadians(constants.upperLimitRots()),
                false,
                0
        );

        this.hoodMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());
        this.motorConfig = new TalonFXConfiguration();

        this.hoodTalonFXSim = new TalonFXSim(
                hoodMotor,
                constants.gearing(),
                armSim::update,
                armSim::setInputVoltage,
                armSim::getAngleRads,
                armSim::getVelocityRadPerSec
        );

        this.hoodPosition = hoodMotor.getPosition(false);
        this.hoodVelocity = hoodMotor.getVelocity(false);
        this.hoodVoltage = hoodMotor.getMotorVoltage(false);
        this.hoodTorqueCurrent = hoodMotor.getTorqueCurrent(false);
        this.hoodDeviceTemp = hoodMotor.getDeviceTemp(false);

        this.positionVoltage = new PositionVoltage(0).withSlot(0);

        RefreshAll.add(
                constants.CANBus(),
                hoodPosition,
                hoodVelocity,
                hoodVoltage,
                hoodTorqueCurrent,
                hoodDeviceTemp
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            hoodTalonFXSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d)",
                hoodMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(0.35)
                .withKG(0.03)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(200)
                .withKD(0);
        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.gearing();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodMotor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                hoodPosition,
                hoodVelocity,
                hoodVoltage,
                hoodTorqueCurrent
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                hoodDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                4,
                hoodMotor
        );

        hoodMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }

    @Override
    public void updateInputs(final HoodIOInputs inputs) {
        inputs.hoodPositionRots = hoodPosition.getValueAsDouble();
        inputs.hoodVelocityRotsPerSec = hoodVelocity.getValueAsDouble();
        inputs.hoodVoltage = hoodVoltage.getValueAsDouble();
        inputs.hoodTorqueCurrentAmps = hoodTorqueCurrent.getValueAsDouble();
        inputs.hoodTempCelsius = hoodDeviceTemp.getValueAsDouble();
    }

    @Override
    public void toHoodPosition(final double positionRots) {
        hoodMotor.setControl(positionVoltage.withPosition(positionRots));
    }

    @Override
    public void zeroMotor() {
        Phoenix6Utils.reportIfNotOk(hoodMotor, hoodMotor.setPosition(0));
    }
}
