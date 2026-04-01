package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;
public enum AllianceShift {
    AUTO,
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    ENDGAME,
    UNKNOWN;

    public enum HubStatus {
        ACTIVE,
        INACTIVE,
        UNKNOWN
    }

    public static final String LogKey = "AllianceShift";

    public HubStatus hubStatus() {
        if (this == AUTO || this == TRANSITION || this == ENDGAME) {
            return HubStatus.ACTIVE;
        } else if (this == UNKNOWN) {
            return HubStatus.UNKNOWN;
        }

        final String gameMessage = DriverStation.getGameSpecificMessage();
        if (gameMessage.isEmpty()) {
            return HubStatus.UNKNOWN;
        }

        final Optional<DriverStation.Alliance> maybeAlliance = DriverStation.getAlliance();
        if (maybeAlliance.isEmpty()) {
            return HubStatus.UNKNOWN;
        }

        final DriverStation.Alliance alliance = maybeAlliance.get();
        final DriverStation.Alliance autoWinningAlliance;
        switch (gameMessage.charAt(0)) {
            case 'R' -> autoWinningAlliance = DriverStation.Alliance.Red;
            case 'B' -> autoWinningAlliance = DriverStation.Alliance.Blue;
            default -> {
                return HubStatus.UNKNOWN;
            }
        }

        final boolean wonAuto = alliance == autoWinningAlliance;
        return switch (this) {
            case SHIFT_1, SHIFT_3 -> wonAuto ? HubStatus.INACTIVE : HubStatus.ACTIVE;
            case SHIFT_2, SHIFT_4 -> wonAuto ? HubStatus.ACTIVE : HubStatus.INACTIVE;
            default -> HubStatus.UNKNOWN;
        };
    }

    public static AllianceShift get(final double matchTimeOffset) {
        if (DriverStation.isAutonomousEnabled()) {
            return AUTO;
        } else if (DriverStation.isDisabled()) {
            return UNKNOWN;
        }

        if (DriverStation.isFMSAttached()) {
            final double matchTime = DriverStation.getMatchTime() - matchTimeOffset;
            if (matchTime > 130) {
                return TRANSITION;
            } else if (matchTime > 105) {
                return SHIFT_1;
            } else if (matchTime > 80) {
                return SHIFT_2;
            } else if (matchTime > 55) {
                return SHIFT_3;
            } else if (matchTime > 30) {
                return SHIFT_4;
            } else {
                return ENDGAME;
            }
        } else {
            final double matchTime = DriverStation.getMatchTime() + matchTimeOffset;
            if (matchTime > 110) {
                return ENDGAME;
            } else if (matchTime > 85) {
                return SHIFT_4;
            } else if (matchTime > 60) {
                return SHIFT_3;
            } else if (matchTime > 35) {
                return SHIFT_2;
            } else if (matchTime > 10) {
                return SHIFT_1;
            } else {
                return TRANSITION;
            }
        }
    }
}
