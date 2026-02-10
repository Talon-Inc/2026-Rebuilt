// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public final class Configs {
  public static final class Algae {
    public static final SparkMaxConfig algaeConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the algae motor
      algaeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
    }
  }

  public static final class Climber {
    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the climber motor
      climberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
    }
  }

  public static final class Drive {
    public static final SparkMaxConfig frontLeftConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rearLeftConfig = new SparkMaxConfig();
    public static final SparkMaxConfig frontRightConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rearRightConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the drive motors
      frontLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);

      rearLeftConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .voltageCompensation(12)
          .follow(DriveConstants.kFrontLeftDriveCanId);

      frontRightConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .voltageCompensation(12)
          .inverted(true);

      rearRightConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .voltageCompensation(12)
          .follow(DriveConstants.kFrontRightDriveCanId);
    }
  }

  public static final class Shooter {
    public static final SparkMaxConfig leftShooter = new SparkMaxConfig();
    public static final SparkMaxConfig rightShooter = new SparkMaxConfig();

    static {
      // Configure basic settings of the left shooter motor
      leftShooter
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .voltageCompensation(12)
          .inverted(true);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      // leftShooter
      //     .closedLoop
      //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //     // Set PID values for position control
      //     .p(0.1)
      //     .i(0)
      //     .d(0)
      //     .outputRange(-1, 1)
      //     .maxMotion
      //     // Set MAXMotion parameters for position control
      //     .maxVelocity(4200)
      //     .maxAcceleration(6000)
      //     .allowedClosedLoopError(0.5);

      // Configure basic settings of the right shooter motor; invert the follower
      rightShooter
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .voltageCompensation(12)
          .follow(ShooterConstants.kLeftShooterCanId, true);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      // rightShooter
      //     .closedLoop
      //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //     // Set PID values for position control
      //     .p(0.1)
      //     .i(0)
      //     .d(0)
      //     .outputRange(-1, 1)
      //     .maxMotion
      //     // Set MAXMotion parameters for position control
      //     .maxVelocity(4200)
      //     .maxAcceleration(6000)
      //     .allowedClosedLoopError(0.5);
    }
  }
}
