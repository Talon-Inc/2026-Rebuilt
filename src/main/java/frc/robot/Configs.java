// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


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
      deployConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .voltageCompensation(12)
          .inverted(false);
      // configs for absolute encoder 

      deployConfig.absoluteEncoder
        .setSparkMaxDataPortConfig() // Tells Spark Max to use the data port
        .inverted(false)             
        .positionConversionFactor(1.0) 
        .zeroOffset(0.0);           

      // Configure basic settings of the intake motor
      intakeConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(30)
          .voltageCompensation(12)
          .inverted(false);
    }
  }

  public static final class ShooterConfigs {
    public static final SparkMaxConfig feederMotor = new SparkMaxConfig();
    public static final SparkMaxConfig topShooter = new SparkMaxConfig();
    public static final SparkMaxConfig bottomShooter = new SparkMaxConfig();
    
    static {
      // Configure basic settings of the feeder motor
      feederMotor
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(50)
          .voltageCompensation(12)
          .inverted(false);

      // Configure basic settings of the top shooter motor
      topShooter
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .voltageCompensation(12)
          .inverted(true);


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
      //     // Set MAXMotion parameters for position control
      //     .maxVelocity(4200)
      //     .maxAcceleration(6000)
      //     .allowedClosedLoopError(0.5);

      // Configure basic settings of the right shooter motor; invert the follower
      // bottomShooter
      //     .idleMode(IdleMode.kBrake)
      //     .smartCurrentLimit(50)
      //     .voltageCompensation(12)
      //     .follow((int) Constants.ShooterConstants.leftShooterCanId, true);

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
  

