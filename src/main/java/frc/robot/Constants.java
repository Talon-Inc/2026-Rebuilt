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

  public static final class IntakeConstants {
    // Intake IDs
    public static final int kDeployMotorId = 11;
    public static final int kIntakeMotorId = 12;

    // PID and feed forward values
    public static final double kP = 0.00125;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kG = -0.6;
  }

  public static final class ShooterConstants {
    // --- Device IDs ---
    // Kicker ID
    public static final int kKickerId = 13;

    // Shooter IDs
    public static final int kTopMotorId = 14;
    public static final int kBottomMotorId = 15;

    // --- Flywheel Constants ---
    public static final double kFlywheelToleranceRPM = 200.0;
    public static final boolean kTopInverted = true;

    // PID and feed forward values, if applicable [top, bottom]
    public static final double kP = 0.000325;
    public static final double kI = 0.0;
    public static final double kD = 0.015;
    public static final double[] kS = {0.1425, 0.112};
    public static final double[] kV = {0.00175, 0.00175};
  }

  public static final class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(651.22);
    public static final double fieldWidth = Units.inchesToMeters(317.69);

    // Passing Gaps
    public static final Translation2d bluePassLeft = new Translation2d(3.0, 6.5);
    public static final Translation2d bluePassRight = new Translation2d(3.0, 1.5);

    // Bottom left of field is (0, 0)
    public static final class Goals {
      public static final Translation2d blueHub =
          new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
      public static final Translation2d redHub =
          new Translation2d(
              fieldLength - Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
    }
  }
}
