// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
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

  // Getters for the top and bottom motors
  public SparkBase getTopMotor();

  public SparkBase getBottomMotor();

  // Updates the set of loggable inputs
  public default void updateInputs(ShooterIOInputs inputs) {}

  // Run the kicker (Speed)
  public default void setKickerSpeed(double speed) {}

  // Run the shooters (RPM)
  public default void setTopRPM(double rpm) {}

  public default void setBottomRPM(double rpm) {}

  // Configure PID Constants (Optional if tuning via Advantage Scope)
  public default void updatePID(
      SparkBase motor,
      SparkBaseConfig config,
      double kP,
      double kI,
      double kD,
      double kS,
      double kV) {}
}
