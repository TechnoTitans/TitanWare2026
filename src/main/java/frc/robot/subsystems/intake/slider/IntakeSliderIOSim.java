package frc.robot.subsystems.intake.slider;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
import frc.robot.utils.sim.feedback.SimCANCoder;
import frc.robot.utils.sim.motors.TalonFXSim;

public class IntakeSliderIOSim implements IntakeSliderIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.IntakeSliderConstants constants;
    
    private final TalonFX sliderMotor;
    private final CANcoder sliderEncoder;
    
    private final TalonFXSim sliderMotorSim;

    private final PositionVoltage positionVoltage;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> sliderPosition;
    private final StatusSignal<AngularVelocity> sliderVelocity;
    private final StatusSignal<Voltage> sliderVoltage;
    private final StatusSignal<Current> sliderTorqueCurrent;
    private final StatusSignal<Temperature> sliderDeviceTemp;
    private final StatusSignal<Angle> sliderCANCoderPosition;
    private final StatusSignal<AngularVelocity> sliderCANCoderVelocity;

    public IntakeSliderIOSim(final HardwareConstants.IntakeSliderConstants constants){
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        this.sliderMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());
        this.sliderEncoder = new CANcoder(constants.encoderID(), constants.CANBus().toPhoenix6CANBus());

        final DCMotorSim sliderSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        1 / (2 * Math.PI),
                        0.1 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(0)
        );

        this.sliderMotorSim = new TalonFXSim(
                sliderMotor,
                constants.gearing(),
                sliderSim::update,
                sliderSim::setInputVoltage,
                sliderSim::getAngularPositionRad,
                sliderSim::getAngularVelocityRadPerSec
        );

        this.sliderMotorSim.attachFeedbackSensor(new SimCANCoder(sliderEncoder));

        this.positionVoltage = new PositionVoltage(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this.sliderPosition = sliderMotor.getPosition(false);
        this.sliderVelocity = sliderMotor.getVelocity(false);
        this.sliderVoltage = sliderMotor.getMotorVoltage(false);
        this.sliderTorqueCurrent = sliderMotor.getTorqueCurrent(false);
        this.sliderDeviceTemp = sliderMotor.getDeviceTemp(false);
        this.sliderCANCoderPosition = sliderEncoder.getPosition(false);
        this.sliderCANCoderVelocity = sliderEncoder.getVelocity(false);

        RefreshAll.add(
                constants.CANBus(),
                sliderPosition,
                sliderVelocity,
                sliderVoltage,
                sliderTorqueCurrent,
                sliderCANCoderPosition,
                sliderCANCoderVelocity
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            sliderMotorSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d)",
                sliderMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = constants.encoderOffset();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        sliderEncoder.getConfigurator().apply(encoderConfig);

        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0 = new Slot0Configs()
                .withKP(1);
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = sliderEncoder.getDeviceID();
        motorConfig.Feedback.RotorToSensorRatio = constants.gearing();
        motorConfig.Feedback.SensorToMechanismRatio = 1;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        sliderMotor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                sliderPosition,
                sliderVelocity,
                sliderVoltage,
                sliderTorqueCurrent,
                sliderCANCoderPosition,
                sliderCANCoderVelocity
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                sliderDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                sliderMotor,
                sliderEncoder
        );

        sliderEncoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        sliderMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    }

    @Override
    public void updateInputs(final IntakeArmIOInputs inputs) {
        inputs.sliderPositionRots = sliderPosition.getValueAsDouble();
        inputs.sliderVelocityRotsPerSec = sliderVelocity.getValueAsDouble();
        inputs.sliderVoltage = sliderVoltage.getValueAsDouble();
        inputs.sliderTorqueCurrentAmps = sliderTorqueCurrent.getValueAsDouble();
        inputs.sliderTempCelsius = sliderDeviceTemp.getValueAsDouble();

        inputs.encoderPositionRots = sliderCANCoderPosition.getValueAsDouble();
        inputs.encoderVelocityRotsPerSec = sliderCANCoderVelocity.getValueAsDouble();
    }

    @Override
    public void toPosition(double positionRots) {
        sliderMotor.setControl(positionVoltage.withPosition(positionRots));
    }
}
