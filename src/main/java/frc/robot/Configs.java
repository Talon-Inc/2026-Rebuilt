// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public final class Configs {
  public static final class ClimberConfigs {
    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the climber motor
      climberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
    }
  }

  public static final class IntakeConfigs {
    public static final SparkMaxConfig deployConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the deploy motor
      // and its absolute encoder
      deployConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .voltageCompensation(12)
          .inverted(false);
      deployConfig
          .absoluteEncoder
          .setSparkMaxDataPortConfig() // Tells Spark Max to use the data port
          .inverted(false)
          .positionConversionFactor(360) // Convert to degrees
          .zeroOffset(0.0);
      deployConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .p(IntakeConstants.kP)
          .i(IntakeConstants.kI)
          .d(IntakeConstants.kD)
          .feedForward // https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control
          .kS(IntakeConstants.kS)
          .kV(IntakeConstants.kV);

      // Configure basic settings of the intake motor
      intakeConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(30)
          .voltageCompensation(12)
          .inverted(true);
    }
  }

  public static final class ShooterConfigs {
    public static final SparkMaxConfig kickerConfig = new SparkMaxConfig();
    public static final SparkFlexConfig leaderConfig = new SparkFlexConfig();
    public static final SparkFlexConfig followerConfig = new SparkFlexConfig();

    static {
      // Configure basic settings of the kicker/feeder motor
      kickerConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(50)
          .voltageCompensation(12)
          .inverted(true);

      // Configure basic settings of the leader motor
      leaderConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(60) // High current limit for Bang-Bang Recovery
          .voltageCompensation(12)
          .inverted(false);

      // Configure basic settings of the follower motor
      // Added follow in ShooterIOReal
      followerConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(60)
          .voltageCompensation(12);
      // .follow(primaryLeader, true);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      // topShooter
      //     .closedLoop
      //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //     // Set PID values for position control
      //     .p(0.1)
      //     .i(0)
      //     .d(0)
      //     .outputRange(-1, 1)
      //     .maxMotion
      //     // Set MAXVelocity parameters for velocity control
      //     .maxVelocity(4200)
      //     .maxAcceleration(6000)
      //     .allowedClosedLoopError(0.5);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      // bottomShooter
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
