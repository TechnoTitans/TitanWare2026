package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;

public class IntakeIOReal implements IntakeIO {
    private final HardwareConstants.IntakeConstants constants;

    private final TalonFX rollerMotor;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerTemperature;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    public IntakeIOReal(final HardwareConstants.IntakeConstants constants) {
        this.constants = constants;

        this.rollerMotor = new TalonFX(constants.motorID(), constants.CANBus().toPhoenix6CANBus());

        this.rollerPosition = rollerMotor.getPosition(false);
        this.rollerVelocity = rollerMotor.getVelocity(false);
        this.rollerVoltage = rollerMotor.getMotorVoltage(false);
        this.rollerTorqueCurrent = rollerMotor.getTorqueCurrent(false);
        this.rollerTemperature = rollerMotor.getDeviceTemp(false);

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
    }

    @Override
    public void updateInputs(final IntakeIOInputs inputs) {
        inputs.rollerPositionRots = rollerPosition.getValueAsDouble();
        inputs.rollerVelocityRotsPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollerTemperatureCelsius = rollerTemperature.getValueAsDouble();
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        rollerMotor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void toRollerVelocity(final double velocityRotsPerSec) {
        rollerMotor.setControl(velocityTorqueCurrentFOC.withVelocity(velocityRotsPerSec));
    }

    @Override
    public void toRollerVoltage(final double volts) {
        rollerMotor.setControl(voltageOut.withOutput(volts));
    }

}
