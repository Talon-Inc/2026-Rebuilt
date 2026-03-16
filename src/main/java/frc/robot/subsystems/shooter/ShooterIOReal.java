// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs.ShooterConfigs;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {
  // Kicker motor
  private final SparkMax kicker;

  // Motors
  private final SparkFlex topMotor;
  private final SparkFlex bottomMotor;

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
        ShooterConfigs.leaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // ShooterConfigs.followerConfig.follow(topMotor, true);
    bottomMotor.configure(
        ShooterConfigs.followerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // -- LOGIC --

  // Update Inputs functions as a way to update the Values in the Logging framework from the REAL
  // sensor values
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Primary Flywheel
    // inputs.[value name] sets the value for the variables you created in ShooterIO
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
  public void setTopVolts(double volts) {
    topMotor.setVoltage(volts);
  }

  @Override
  public void setBottomVolts(double volts) {
    bottomMotor.setVoltage(volts);
  }
}
