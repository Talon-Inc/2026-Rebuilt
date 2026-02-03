// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
    // Drive motors - SparkMax Controller
    public static final int kFrontLeftDriveCanId = 1;
    public static final int kRearLeftDriveCanId = 3;
    public static final int kFrontRightDriveCanId = 2;
    public static final int kRearRightDriveCanId = 4;
    public static final MotorType kDriveMotorType = MotorType.kBrushed;
    public static final double kSpeed = .5;
    public static final IdleMode kDriveIdleMode = IdleMode.kBrake;
    public static final int kDriveMotorCurrentLimit = 30;
    public static double kDriveFactor = .5; // changed for turbo
    public static double kTurnFactor = .5;
    public static int kDriveReverse = 1; // Default drive; 1 = Front is panel, -1 = Front is intake
  }

  public static final class ShooterConstants {
    // SparkMax Controllers
    public static final int kLeftShooterCanId = 11;
    public static final int kRightShooterCanId = 12;
    public static final MotorType kShooterMotorType = MotorType.kBrushed;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
