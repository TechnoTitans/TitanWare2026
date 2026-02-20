package frc.robot.utils.sim.feedback;

import frc.robot.utils.sim.motors.TalonFXSim;

public class CRTSim {
    private final SimCANCoder encoder1;
    private final SimCANCoder encoder2;
    private final TalonFXSim motorSim;

    private final double encoder1Gearing;
    private final double encoder2Gearing;

    public CRTSim(
            final SimCANCoder encoder1,
            final SimCANCoder encoder2,
            final TalonFXSim motorSim,
            final double encoder1Gearing,
            final double encoder2Gearing,
            final double turretTooth
    ) {
        this.encoder1 = encoder1;
        this.encoder2 = encoder2;
        this.motorSim = motorSim;

        this.encoder1Gearing = turretTooth / encoder1Gearing;
        this.encoder2Gearing = turretTooth / encoder2Gearing;
    }

    public void update() {
        final double mechanismPositionRots = motorSim.getAngularPositionRots();
        final double mechanismVelocityRotsPerSec = motorSim.getAngularVelocityRotsPerSec();

        encoder1.setRawPosition(mechanismPositionRots * encoder1Gearing);
        encoder1.setVelocity(mechanismVelocityRotsPerSec * encoder1Gearing);

        encoder2.setRawPosition(mechanismPositionRots * encoder2Gearing);
        encoder2.setVelocity(mechanismVelocityRotsPerSec * encoder2Gearing);
    }
}