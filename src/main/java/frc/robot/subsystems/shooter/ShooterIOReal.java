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
  
  // Primary Set
  private final SparkFlex primaryLeader;
  private final SparkFlex primaryFollower;

  // Secondary Set (Optional)
  private final SparkFlex secondaryLeader;
  private final SparkFlex secondaryFollower;

  public ShooterIOReal() {
    // --- 1. Kicker Setup ---
    kicker = new SparkMax(ShooterConstants.kKickerId, MotorType.kBrushless);

    // Configure Kicker/Feeder
    kicker.configure(ShooterConfigs.kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- 2. Primary Flywheel Setup ---
    primaryLeader = new SparkFlex(ShooterConstants.kPrimaryLeaderId, MotorType.kBrushless);
    primaryFollower = new SparkFlex(ShooterConstants.kPrimaryFollowerId, MotorType.kBrushless);

    // Configure Leader
    primaryLeader.configure(ShooterConfigs.leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure Follower (Follows Leader)
    ShooterConfigs.followerConfig.follow(primaryLeader, true);
    primaryFollower.configure(ShooterConfigs.followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- 3. Secondary Flywheel Setup ---
    if (ShooterConstants.kIsDoubleFlywheel) {
      secondaryLeader = new SparkFlex(ShooterConstants.kSecondaryLeaderId, MotorType.kBrushless);
      secondaryFollower = new SparkFlex(ShooterConstants.kSecondaryFollowerId, MotorType.kBrushless);

      secondaryLeader.configure(ShooterConfigs.leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      ShooterConfigs.followerConfig.follow(secondaryLeader, true);
      secondaryFollower.configure(ShooterConfigs.followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } else {
      secondaryLeader = null;
      secondaryFollower = null;
    }
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
    
    // Secondary Flywheel
    if (secondaryLeader != null) {
      inputs.secondaryLeaderRPM = secondaryLeader.getEncoder().getVelocity();
      inputs.secondaryLeaderVolts =
          secondaryLeader.getAppliedOutput() * secondaryLeader.getBusVoltage();
      inputs.secondaryLeaderAmps = new double[] {secondaryFollower.getOutputCurrent()};

      inputs.secondaryFollowerVolts =
          secondaryFollower.getAppliedOutput() * secondaryFollower.getBusVoltage();
      inputs.secondaryFollowerAmps = new double[] {secondaryFollower.getOutputCurrent()};
    }
  }

  @Override
  public void setKickerSpeed(double speed) {
    kicker.set(speed);
  }

  @Override
  public void setPrimaryVolts(double volts) {
    primaryLeader.setVoltage(volts);
  }

  @Override
  public void setSecondaryVolts(double volts) {
    secondaryLeader.setVoltage(volts);
  }
}
