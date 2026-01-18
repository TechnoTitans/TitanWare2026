package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class FieldConstants {
    public static final double FIELD_LENGTH_X_METERS = Units.inchesToMeters(690.876);
    public static final double FIELD_WIDTH_Y_METERS = Units.inchesToMeters(317);


    public static final Translation2d hubCenter = new Translation2d(4.6256194, 4.0346376);

    public static final Translation2d ferryTarget = new Translation2d(2, 2);

}