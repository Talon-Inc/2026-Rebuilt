package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class AllianceFlipUtil {
  // This checks if tare on the Red Alliance
  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  // Flips the X coordinates if we are on the Red Side
  public static Translation2d apply(Translation2d translation) {
    if (isRedAlliance()) {
      // Subtract x from teh total length to mirror it (y doesn't change)
      return new Translation2d(FieldConstants.fieldLength - translation.getX(), translation.getY());
    }
    return translation;
  }
}
