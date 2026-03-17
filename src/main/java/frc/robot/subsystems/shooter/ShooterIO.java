// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // --- Kicker ---
    public double kickerVolts = 0.0;
    public double[] kickerAmps = new double[] {};

    // --- Top Motor ---
    public double topRPM = 0.0;
    public double topVolts = 0.0;
    public double[] topAmps = new double[] {};

    // --- Bottom Roller ---
    public double bottomRPM = 0.0;
    public double bottomVolts = 0.0;
    public double[] bottomAmps = new double[] {};
  }

  // Updates the set of loggable inputs
  public default void updateInputs(ShooterIOInputs inputs) {}

  // Run the kicker (Volts)
  public default void setKickerSpeed(double speed) {}

  // Run the shooters (Volts)
  public default void setTopVolts(double volts) {}

  public default void setBottomVolts(double volts) {}

  // Configure PID Constants (Optional If tuning Via Advantage Scope)
  public default void configurePID(double kP, double kI, double kD) {}
}
