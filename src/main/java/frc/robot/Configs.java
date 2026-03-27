// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

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
          .smartCurrentLimit(60)
          .voltageCompensation(12)
          .inverted(false);
      deployConfig
          .absoluteEncoder
          .setSparkMaxDataPortConfig() // Tells Spark Max to use the data port
          .inverted(false)
          .positionConversionFactor(360) // Convert to degrees
          .zeroOffset(
              (IntakeConstants.kStartAngle - 2) / 360); // Reset zero value to a bit behind stow
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
    public static final SparkFlexConfig topConfig = new SparkFlexConfig();
    public static final SparkFlexConfig bottomConfig = new SparkFlexConfig();

    static {
      // Configure basic settings of the kicker/feeder motor
      kickerConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(50)
          .voltageCompensation(12)
          .inverted(true);

      // Configure basic settings of the top motor
      topConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(60)
          .voltageCompensation(12)
          .inverted(ShooterConstants.kTopInverted);

      // Configure basic settings of the bottom motor
      bottomConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(60)
          .voltageCompensation(12)
          // invert compared to top motor
          .inverted(!ShooterConstants.kTopInverted);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      topConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for velocity control
          .p(ShooterConstants.kP)
          .i(ShooterConstants.kI)
          .d(ShooterConstants.kD)
          .outputRange(-1, 1)
          .feedForward // https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control
          .kS(ShooterConstants.kS[0])
          .kV(ShooterConstants.kV[0]);
      // topConfig
      //     .closedLoop
      //     .maxMotion // Set MAXMotion parameters for velocity control
      //     .maxAcceleration(ShooterConstants.kMaxAcceleration)
      //     .allowedProfileError(ShooterConstants.kError);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      bottomConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for velocity control
          .p(ShooterConstants.kP)
          .i(ShooterConstants.kI)
          .d(ShooterConstants.kD)
          .outputRange(-1, 1)
          .feedForward // https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control
          .kS(ShooterConstants.kS[1])
          .kV(ShooterConstants.kV[1]);
      // bottomConfig
      //     .closedLoop
      //     .maxMotion // Set MAXMotion parameters for velocity control
      //     .maxAcceleration(ShooterConstants.kMaxAcceleration)
      //     .allowedProfileError(ShooterConstants.kError);
    }
  }
}
