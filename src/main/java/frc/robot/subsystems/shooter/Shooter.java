// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // Flywheel Controllers (These use Bang-Bang bc it's the quickest)
  // Bang-Bang Controller: If Under speed, Full Power. If over power, 0 power
  private final BangBangController topBang;
  private final BangBangController bottomBang;

  // Setpoints
  private double targetTopRPM = 0.0;
  private double targetBottomRPM = 0.0;
  private double targetKickerRPM = 0.0;

  // Tunable Numbers
  // Flywheel Power (Bang-Bang Voltage)
  private final LoggedTunableNumber flyVolts =
      new LoggedTunableNumber("Tuning/Shooter/FlywheelVolts", ShooterConstants.kBangBangVoltage);

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;

    // Initialize Bang-Bang Controllers
    topBang = new BangBangController(ShooterConstants.kFlywheelToleranceRPM);
    bottomBang = new BangBangController(ShooterConstants.kFlywheelToleranceRPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Refresh Inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Update Tunables (This is for Live Tuning should make tuning faster)
    // Run Flywheel Logic
    runFlywheel(topBang, inputs.topRPM, targetTopRPM, true, flyVolts.get());
    runFlywheel(bottomBang, inputs.bottomRPM, targetBottomRPM, false, flyVolts.get());

    // Log Goals
    Logger.recordOutput("Shooter/Goal/PrimaryRPM", targetTopRPM);
    Logger.recordOutput("Shooter/Goal/SecondaryRPM", targetBottomRPM);
    Logger.recordOutput("Shooter/Goal/KickerRPM", targetKickerRPM);
  }

  // Core Bang-Bang Logic
  private void runFlywheel(
      BangBangController controller,
      double currentRPM,
      double target,
      boolean isTop,
      double voltageToUse) {
    // Only run if we actualy have a target (avoids jitter at 0 RPM)
    if (target > 50.0) {
      // .calculate() returns 1.0 if we need speed, 0.0 if we don't
      double demand = controller.calculate(currentRPM, target);

      // Multiply by Max Voltage (12V)
      double volts = demand * voltageToUse;

      if (isTop) io.setTopVolts(volts);
      else io.setBottomVolts(volts);
    } else {
      // Idle Mode
      if (isTop) io.setTopVolts(0.0);
      else io.setBottomVolts(0.0);
    }
  }

  // --- Commands ---

  // This will set rollers to the exact same speeds (Natural Backspin)
  public void setTargetSpeed(double rpm) {
    this.targetTopRPM = rpm;
    this.targetBottomRPM = rpm;
  }

  // Sets different speeds for Top and Bottom Rollers
  // This is useful  for controlling spin (backspin/topspin)
  public void setSplitSpeeds(double bottomRPM, double topRPM) {
    this.targetBottomRPM = bottomRPM;
    this.targetTopRPM = topRPM;
  }

  /**
   * @return True if flywheels are within tolerance of target
   */
  public boolean isAtSpeed() {
    boolean topReady =
        Math.abs(inputs.topRPM - targetTopRPM) < ShooterConstants.kFlywheelToleranceRPM;

    boolean bottomReady =
        Math.abs(inputs.bottomRPM - targetBottomRPM) < ShooterConstants.kFlywheelToleranceRPM;

    return topReady && bottomReady;
  }

  // kicker (wheels that feed into the shooter)
  public void setKickerSpeed(double speed) {
    io.setKickerSpeed(speed);
  }

  public void setMotorVoltage(double voltage) {
    io.setTopVolts(voltage);
    io.setBottomVolts(voltage);
  }
}
