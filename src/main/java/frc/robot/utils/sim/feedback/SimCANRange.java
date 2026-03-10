package frc.robot.utils.sim.feedback;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.sim.CANrangeSimState;
import frc.robot.utils.ctre.Phoenix6Utils;

public class SimCANRange implements SimFeedbackSensor {
    private final CANrange canRange;
    private final CANrangeSimState simState;

    private final double rotationsToLinearDistanceRatio;

    public SimCANRange(final CANrange canRange, final double rotationsToLinearDistanceRatio) {
        this.canRange = canRange;
        this.simState = canRange.getSimState();

        this.rotationsToLinearDistanceRatio = rotationsToLinearDistanceRatio;
    }

    @Override
    public void setSupplyVoltage(final double volts) {
        Phoenix6Utils.reportIfNotOk(canRange, simState.setSupplyVoltage(volts));
    }

    @Override
    public void setRawPosition(final double rotations) {
        Phoenix6Utils.reportIfNotOk(canRange, simState.setDistance(rotations * rotationsToLinearDistanceRatio));
    }
}
