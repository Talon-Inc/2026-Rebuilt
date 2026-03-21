package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
// import frc.robot.Constants.ShooterConstants;

public class ShootingPhysics {

  // Key: Distance (m), Value: Time of Flight (s)
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Key: Distance (m), Value: RPM
  private static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

  // Key: Distance (m), Value: Hood Angle (Degrees)
  private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap bottomRPMMap = new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap topRPMMap = new InterpolatingDoubleTreeMap();

  // PASSING MAPS
  private static final InterpolatingDoubleTreeMap passTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passTopRPM = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passBottomRPM = new InterpolatingDoubleTreeMap();

  static {
    // The More the data you put the more accurate it's going to be
    // But don't put too much

    // Scoring Data
    // Time of flight (this can be tuned by recording in slo-mo)
    timeOfFlightMap.put(1.0, .2);
    timeOfFlightMap.put(5.0, 1.0);

    // Key: Distance(m), Value:RPM
    bottomRPMMap.put(1.549, 2500.0);
    bottomRPMMap.put(2.667, 4000.0);
    bottomRPMMap.put(5.0, 5000.0);

    // Key: Distance(m), Value:RPM
    topRPMMap.put(1.549, 2500.0);
    topRPMMap.put(2.667, 4000.0);
    topRPMMap.put(5.0, 5000.0);

    // Passing Data
    passTimeOfFlightMap.put(2.0, 0.4);

    passTimeOfFlightMap.put(8.0, 1.2);

    passTopRPM.put(2.0, 1000.0);
    passTopRPM.put(8.0, 2000.0);

    passBottomRPM.put(2.0, 2000.0);
    passBottomRPM.put(8.0, 4000.0);
  }

  public enum ShotType {
    SCORE,
    PASS
  }

  public record ShootSolution(
      Rotation2d robotHeading, // Field-Relative Robot/Turret Angle
      double topRPM, // Adjusted Top RPM
      double bottomRPM,
      double effectiveDist // This is for debugging
      ) {}

  /**
   * Calculates the shooting vector based on robot motion. Implements "V_ball/robot = V_ball/ground
   * - V_robot/ground" * @param currentPose Current robot pose (from Odometry/Vision)
   *
   * @param fieldRelativeSpeeds Current robot velocity (Field Relative!)
   * @param targetLocation Location of the goal (Speaker)
   * @return ShotSolution containing the field-relative angle and the necessary shooter speed
   */
  public static ShootSolution calculateShot(
      Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetLocation) {

    // 1. Get the initial distance
    double currentDistance = targetLocation.getDistance(currentPose.getTranslation());

    // 2. This is going to run the time of fight stuff multiple times for accuracy
    double timeOffFlight = 0.0;
    Translation2d virtualGoalLocation = targetLocation;
    double effectiveDistance = currentDistance;

    // Run 5 passes (this can be changed depending if we want more accuracy)
    for (int i = 0; i < 5; i++) {
      // Use the look up table for the current guess distance
      timeOffFlight = timeOfFlightMap.get(effectiveDistance);

      // Calculate the virtual goal position
      // This matters because if we are moving 4m/s away from the goal, and the shot takes 1 second
      // the goal moves 4 meters further away relative to use during the shot
      double virtualGoalX =
          targetLocation.getX() - (fieldRelativeSpeeds.vxMetersPerSecond * timeOffFlight);
      double virtualGoalY =
          targetLocation.getY() - (fieldRelativeSpeeds.vyMetersPerSecond * timeOffFlight);

      virtualGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

      // Update the distance for the next loop
      effectiveDistance = virtualGoalLocation.getDistance(currentPose.getTranslation());
    }

    // I deleted the past physics that used latency because it took up too much space

    // This handles leading the shot & rotation
    Rotation2d targetHeading = virtualGoalLocation.minus(currentPose.getTranslation()).getAngle();

    // RPM & Hood: This part tells the shooter to shoot harder/higher
    // if we are moving away
    double topRPM = topRPMMap.get(effectiveDistance);
    double bottomRPM = bottomRPMMap.get(effectiveDistance);

    return new ShootSolution(targetHeading, topRPM, bottomRPM, effectiveDistance);
  }

  // Calculate Pass
  public static ShootSolution calculatePass(
      Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetLocation) {
    double currentDistance = targetLocation.getDistance(currentPose.getTranslation());
    double timeOfFlight = 0.0;
    Translation2d virtualGoalLocation = targetLocation;
    double effectiveDistance = currentDistance;

    for (int i = 0; i < 5; i++) {
      timeOfFlight = passTimeOfFlightMap.get(effectiveDistance);
      double virtualGoalX =
          targetLocation.getX() - (fieldRelativeSpeeds.vxMetersPerSecond * timeOfFlight);
      double virtualGoalY =
          targetLocation.getY() - (fieldRelativeSpeeds.vyMetersPerSecond * timeOfFlight);

      virtualGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);
      effectiveDistance = virtualGoalLocation.getDistance(currentPose.getTranslation());
    }

    Rotation2d targetHeading = virtualGoalLocation.minus(currentPose.getTranslation()).getAngle();

    return new ShootSolution(
        targetHeading,
        passTopRPM.get(effectiveDistance),
        passBottomRPM.get(effectiveDistance),
        effectiveDistance);
  }

  // ***DISCLAIMER**
  // THESE ARE OLD CODE; it was used to calculate SOTF using Vectors * latency
  // Mock Lookup Table: Distance (m) -> Horizontal Velocity (m/s)
  @SuppressWarnings("unused")
  private static double getStationaryHorizontalSpeed(double distance) {
    // Example: 6 m/s base  + linear increase
    return 8.0 + (distance * 1.2);
  }

  // Mock Lockup Table: distance(m) -> Hood Angle (rad)
  @SuppressWarnings("unused")
  private static double getIdealHoodAngle(double dist) {
    // Example: closer = Higher shot (60 deg), Farther = Lower shot (30)
    // Linear Interpolation or Interpolation Double Tree Map recommended here
    double angleDeg = 60.0 - (dist * 5.0);
    return Units.degreesToRadians(Math.max(25.0, Math.min(60.0, angleDeg)));
  }
}
