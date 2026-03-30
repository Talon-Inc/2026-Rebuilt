// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // To Deploy Arm
    public double deployAngleDeg = 0.0;
    public double deployVolts = 0.0;
    public double[] deployAmps = new double[] {};

    // Roller
    public double rollerRPM = 0.0;
    public double rollerVolts = 0.0;
    public double[] rollerAmps = new double[] {};
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  // Commands
  public default void setDeployPosition(double angleDeg) {}

  public default void setDeployPosition(double angleDeg, double feedForwardVolts) {}

  public default void setRollerVoltage(double volts) {}

  // PID Updates (this "should" work better we'll see)
  public default void updatePID(double kP, double kI, double kD, double kS, double kV) {}
}
