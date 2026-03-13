// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Configs.ShooterConfigs;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {
  // Primary Set
  private final SparkFlex primaryLeader;
  private final SparkFlex primaryFollower;

  // kicker set
  private final SparkMax kicker;

  public ShooterIOReal() {
    // --- 1. Primary Flywheel Setup ---
    primaryLeader = new SparkFlex(ShooterConstants.kPrimaryLeaderID, MotorType.kBrushless);
    primaryFollower = new SparkFlex(ShooterConstants.kPrimaryFollowerID, MotorType.kBrushless);

    // Configure Leader
    configureLeader(primaryLeader);

    // Configure Follower (Follows Leader, Inverted = true usually for shooters on opposite sides)
    configureFollower(primaryFollower, primaryLeader, true);

    // --- 3. Kicker Setup ---
    kicker = new SparkMax(ShooterConstants.kKickerId, MotorType.kBrushless);
  }

  private void configureLeader(SparkFlex motor) {
    SparkFlexConfig config = new SparkFlexConfig();

    config.idleMode(IdleMode.kCoast);

    // High current limit for Bang-Bang Recovery
    config.smartCurrentLimit(60);
    // Apply Config
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureFollower(SparkFlex follower, SparkFlex leader, boolean inverted) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(60);

    config.follow(leader, inverted);
    follower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // -- LOGIC --

  // Update Inputs functions as a way to update the Values in the Logging framework from the REAL sensor values
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Primary Flywheel
    // inputs.[value name] sets the value for the variables you created in ShooterIO
    inputs.primaryLeaderRPM = primaryLeader.getEncoder().getVelocity();
    inputs.primaryLeaderVolts = primaryLeader.getAppliedOutput() * primaryFollower.getBusVoltage();
    inputs.primaryLeaderAmps = new double[] {primaryFollower.getOutputCurrent()};

    inputs.primaryFollowerVolts =
        primaryFollower.getAppliedOutput() * primaryFollower.getBusVoltage();
    inputs.primaryFollowerAmps = new double[] {primaryFollower.getOutputCurrent()};
  
  }

  @Override
  public void setPrimaryVolts(double volts) {
    primaryLeader.setVoltage(volts);
  }

  @Override
  public void setKickerSpeed(double speed) {
    kicker.set(speed);
  }
}
