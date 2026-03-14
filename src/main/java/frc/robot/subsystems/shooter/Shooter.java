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
  private final BangBangController primaryBang;
  private final BangBangController secondaryBang;

  // Setpoints
  private double targetPrimaryRPM = 0.0;
  private double targetSecondaryPRM = 0.0;
  private double targetKickerRPM = 0.0;

  // Tunable Numbers
  // Flywheel Power (Bang-Bang Voltage)
  private final LoggedTunableNumber flyVolts =
      new LoggedTunableNumber("Tuning/Shooter/FlywheelVolts", ShooterConstants.kBangBangVoltage);

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;

    // Initialize Bang-Bang Controllers
    primaryBang = new BangBangController(ShooterConstants.kFlywheelToleranceRPM);
    secondaryBang = new BangBangController(ShooterConstants.kFlywheelToleranceRPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Refresh Inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Update Tunables (This is for Live Tuning should make tuning faster)
    // Run Flywheel Logic
    runFlywheel(primaryBang, inputs.primaryLeaderRPM, targetPrimaryRPM, true, flyVolts.get());

    // Log Goals
    Logger.recordOutput("Shooter/Goal/PrimaryRPM", targetPrimaryRPM);
    Logger.recordOutput("Shooter/Goal/KickerRPM", targetKickerRPM);
  }

  // Core Bang-Bang Logic
  private void runFlywheel(
      BangBangController controller,
      double currentRPM,
      double target,
      boolean isPrimary,
      double voltageToUse) {
    // Only run if we actualy have a target (aooids jitter at 0 RPM)
    if (target > 50.0) {
      // .calculate() returns 1.0 if we need speed, 0.0 if we don't
      double demand = controller.calculate(currentRPM, target);

      // Multiply by Max Voltage (12V)
      double volts = demand * voltageToUse;

      io.setPrimaryVolts(volts);
    } else {
      // Idle Mode
      io.setPrimaryVolts(0.0);
    }
  }

  // --- Commands ---

  // Set the main target speed
  // If double Flywheel is enabled, this sets BOTH to the same speed
  public void setTargetSpeed(double rpm) {
    this.targetPrimaryRPM = rpm;
    if (ShooterConstants.kIsDoubleFlywheel) {
      this.targetSecondaryPRM = rpm;
    }
  }

  // Sets different speeds for Top and Bottom Rollers
  // Thisi s useful  for controlling spin (backspin/topspin)
  public void setSplitSpeeds(double primaryRPM, double secondaryRPM) {
    this.targetPrimaryRPM = primaryRPM;
    this.targetSecondaryPRM = secondaryRPM;
  }

  /**
   * @return True if flywheels are within tolerance of target
   */
  public boolean isAtSpeed() {
    boolean primaryReady =
        Math.abs(inputs.primaryLeaderRPM - targetPrimaryRPM)
            < ShooterConstants.kFlywheelToleranceRPM;

    if (ShooterConstants.kIsDoubleFlywheel) {
      boolean secondaryReady =
          Math.abs(inputs.secondaryLeaderRPM = targetSecondaryPRM)
              < ShooterConstants.kFlywheelToleranceRPM;
      return primaryReady && secondaryReady;
    }

    return primaryReady;
  }

  // kicker (wheels that feed into the shooter)
  public void setKickerSpeed(double speed) {
    io.setKickerVolts(speed * 12);
  }
}
