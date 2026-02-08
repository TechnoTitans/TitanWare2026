package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.motors.TalonFXSim;

public class ClimbIOSim implements ClimbIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.0;

    private final DeltaTime deltaTime;
    private final HardwareConstants.ClimbConstants constants;
    private final double drumCircumferenceMeters;

    private final ElevatorSim climbSim;

    private final TalonFX climbMotor;
    private final TalonFXSim motorsSim;

    private final PositionVoltage positionVoltage;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    public ClimbIOSim(final HardwareConstants.ClimbConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;
        this.drumCircumferenceMeters = constants.spoolDiameterMeters() * Math.PI;

        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(1);

        final double lowerLimitMeters = constants.lowerLimitRots() * drumCircumferenceMeters;
        final double upperLimitMeters = constants.upperLimitRots() * drumCircumferenceMeters;
        this.climbSim = new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                        dcMotor,
                        SimConstants.Climb.MASS_KG,
                        constants.spoolDiameterMeters() * 0.5,
                        constants.climbGearing()
                ),
                dcMotor,
                lowerLimitMeters,
                upperLimitMeters,
                true,
                lowerLimitMeters
        );

        this.climbMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.motorsSim = new TalonFXSim(
                climbMotor,
                constants.climbGearing(),
                climbSim::update,
                climbSim::setInputVoltage,
                () -> Units.rotationsToRadians(climbSim.getPositionMeters() / drumCircumferenceMeters),
                () -> Units.rotationsToRadians(climbSim.getVelocityMetersPerSecond() / drumCircumferenceMeters)
        );

        this.positionVoltage = new PositionVoltage(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this.motorPosition = climbMotor.getPosition(false);
        this.motorVelocity = climbMotor.getVelocity(false);
        this.motorVoltage = climbMotor.getMotorVoltage(false);
        this.motorTorqueCurrent = climbMotor.getTorqueCurrent(false);
        this.motorDeviceTemp = climbMotor.getDeviceTemp(false);

        RefreshAll.add(
                constants.CANBus(),
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                motorDeviceTemp
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            motorsSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d)",
                climbMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
//                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(100.0)
                .withKD(0.01);
        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.climbGearing();
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.lowerLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        climbMotor.getConfigurator().apply(motorConfiguration);

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
                climbMotor
        );

        climbMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public void updateInputs(final ClimbIOInputs inputs) {
        inputs.motorPositionRots = motorPosition.getValueAsDouble();
        inputs.motorVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.motorVoltage = motorVoltage.getValueAsDouble();
        inputs.motorTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.motorTempCelsius = motorDeviceTemp.getValueAsDouble();
    }

    @Override
    public void setPosition(final double positionRots) {
        Phoenix6Utils.reportIfNotOk(climbMotor, climbMotor.setPosition(positionRots));
    }

    @Override
    public void toPosition(final double positionRots) {
        climbMotor.setControl(positionVoltage.withPosition(positionRots));
    }

    @Override
    public void toVoltage(final double volts) {
        climbMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toTorqueCurrent(final double torqueCurrentAmps) {
        climbMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }
}
