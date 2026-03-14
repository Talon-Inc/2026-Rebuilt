// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8;
  }

  public static final class IntakeConstants {
    // Intake motor confugurations
    public static final int kDeployMotorId = 11;
    public static final int kIntakeMotorId = 12;
  }

  public static final class ShooterConstants {
    public static final double kFixedPitchDegrees = 0;

    // --- Configuration Flags ---
    public static final boolean kHasHood = true;
    public static final boolean kIsDoubleFlywheel = true; // Set TRUE for Top/Bottom shooter
    public static final boolean kHasDualHoods = true;

    // --- Device IDs ---
    // Kicker
    public static final int kKickerId = 13;

    // Primary Flywheel (e.g. Bottom Roller) - 2 Motors
    public static final int kPrimaryLeaderId = 20;
    public static final int kPrimaryFollowerId = 21;

    // Secondary Flywheel (e.g. Top Roller) - 2 Motors
    public static final int kSecondaryLeaderId = 24;
    public static final int kSecondaryFollowerId = 25;

    // --- Flywheel Constants (Bang-Bang) ---
    public static final double kFlywheelToleranceRPM = 50.0;
    public static final double kMaxRPM = 5700.0;
    public static final double kBangBangVoltage = 12.0;
  }

  public static final class FieldConstants {
    public static final double fieldLength = 0.0;
    public static final double fieldWidth = 0.0;

    public static final class Goals {
      public static final Translation2d blueGoal = new Translation2d();
      public static final Translation2d redGoal = new Translation2d();
    }
  }
}
