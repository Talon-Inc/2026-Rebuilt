// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.Configs.ShooterConfigs;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {
  // Kicker Motor
  private final SparkMax kicker;

  // Shooter Motors
  private final SparkFlex topMotor;
  private final SparkFlex bottomMotor;

  // ClosedLoopControllers
  private final SparkClosedLoopController topController;
  private final SparkClosedLoopController bottomController;

  public ShooterIOReal() {
    // --- 1. Kicker Setup ---
    kicker = new SparkMax(ShooterConstants.kKickerId, MotorType.kBrushless);

    // Configure Kicker/Feeder
    kicker.configure(
        ShooterConfigs.kickerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // --- 2. Motor Setup ---
    topMotor = new SparkFlex(ShooterConstants.kTopMotorId, MotorType.kBrushless);
    bottomMotor = new SparkFlex(ShooterConstants.kBottomMotorId, MotorType.kBrushless);

    // Configure Motors
    topMotor.configure(
        ShooterConfigs.topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    bottomMotor.configure(
        ShooterConfigs.bottomConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // --- 3. Controller Setup ---
    topController = topMotor.getClosedLoopController();
    bottomController = bottomMotor.getClosedLoopController();
  }

  /**
   * @return the top shooter motor
   */
  @Override
  public SparkBase getTopMotor() {
    return topMotor;
  }

  /**
   * @return the bottom shooter motor
   */
  @Override
  public SparkBase getBottomMotor() {
    return bottomMotor;
  }

  // -- LOGIC --

  /**
   * Update Inputs functions as a way to update the Values in the Logging framework from the REAL
   * sensor values
   *
   * <p>inputs.[value name] sets the value for the variables you created in ShooterIO
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.topRPM = topMotor.getEncoder().getVelocity();
    inputs.topVolts = topMotor.getAppliedOutput() * topMotor.getBusVoltage();
    inputs.topAmps = new double[] {topMotor.getOutputCurrent()};

    inputs.bottomRPM = bottomMotor.getEncoder().getVelocity();
    inputs.bottomVolts = bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage();
    inputs.bottomAmps = new double[] {bottomMotor.getOutputCurrent()};
  }

  @Override
  public void setKickerSpeed(double speed) {
    kicker.set(speed);
  }

  @Override
  public void setTopRPM(double rpm) {
    topController.setSetpoint(rpm, ControlType.kVelocity);
  }

  @Override
  public void setBottomRPM(double rpm) {
    bottomController.setSetpoint(rpm, ControlType.kVelocity);
  }

  @Override
  public void updatePID(
      SparkBase motor,
      SparkBaseConfig config,
      double kP,
      double kI,
      double kD,
      double kS,
      double kV) {
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP)
        .i(kI)
        .d(kD)
        .feedForward
        .kS(kS)
        .kV(kV);

    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
