// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // --- Primary Set (Bottom) ---
    public double primaryLeaderRPM = 0.0;
    public double primaryLeaderVolts = 0.0;
    public double[] primaryLeaderAmps = new double[] {};

    // Primary Follower Data
    public double primaryFollowerVolts = 0.0;
    public double[] primaryFollowerAmps = new double[] {};

    // --- Secondary Set (Top) ---
    public double secondaryLeaderRPM = 0.0;
    public double secondaryLeaderVolts = 0.0;
    public double[] secondaryLeaderAmps = new double[] {};

    // Secondary Follower Data
    public double secondaryFollowerVolts = 0.0;
    public double[] secondaryFollowerAmps = new double[] {};

    // --- kicker ---
    public double kickerAmps = 0.0;
    public double kickerVolts = 0.0;
  }

  // Updates the set of loggable inputs
  public default void updateInputs(ShooterIOInputs inputs) {}

  // Run the shooters (Volts)
  public default void setPrimaryVolts(double volts) {}

  // Run the kicker (Volts)
  public default void setKickerVolts(double volts) {}

  // Configure PID Constants (Optional If tuning Via Advantage Scope)
  public default void configurePID(double kP, double kI, double kD) {}
}
