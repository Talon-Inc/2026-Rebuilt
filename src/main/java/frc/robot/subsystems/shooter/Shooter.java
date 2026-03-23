// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShooterConfigs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // Setpoints
  private double targetTopRPM = 0.0;
  private double targetBottomRPM = 0.0;
  private double targetKickerRPM = 0.0;

  // Tunable Numbers
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("/Tuning/Shooter/kP", ShooterConstants.kP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("/Tuning/Shooter/kI", ShooterConstants.kI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("/Tuning/Shooter/kD", ShooterConstants.kD);
  private final LoggedTunableNumber kSTop =
      new LoggedTunableNumber("/Tuning/Shooter/kSTop", ShooterConstants.kS[0]);
  private final LoggedTunableNumber kSBottom =
      new LoggedTunableNumber("/Tuning/Shooter/kSBottom", ShooterConstants.kS[1]);
  private final LoggedTunableNumber kVTop =
      new LoggedTunableNumber("/Tuning/Shooter/kVTop", ShooterConstants.kV[0]);
  private final LoggedTunableNumber kVBottom =
      new LoggedTunableNumber("/Tuning/Shooter/kVBottom", ShooterConstants.kV[1]);

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Refresh Inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Update Tunables (This is for Live Tuning should make tuning faster)
    if (kP.hasChanged()
        || kI.hasChanged()
        || kD.hasChanged()
        || kSTop.hasChanged()
        || kSBottom.hasChanged()
        || kVTop.hasChanged()
        || kVBottom.hasChanged()) {
      io.configurePID(
          io.getTopMotor(),
          ShooterConfigs.topConfig,
          kP.get(),
          kI.get(),
          kD.get(),
          kSTop.get(),
          kVTop.get());
      io.configurePID(
          io.getBottomMotor(),
          ShooterConfigs.topConfig,
          kP.get(),
          kI.get(),
          kD.get(),
          kSBottom.get(),
          kVBottom.get());
    }

    io.setTopRPM(targetTopRPM);
    io.setBottomRPM(targetBottomRPM);

    // Log Goals
    Logger.recordOutput("Shooter/Goal/TopRPM", targetTopRPM);
    Logger.recordOutput("Shooter/Goal/BottomRPM", targetBottomRPM);
    Logger.recordOutput("Shooter/Goal/KickerRPM", targetKickerRPM);
  }

  // --- Commands ---

  // Run kicker (wheels that feed into the shooter)
  public void setKickerSpeed(double speed) {
    io.setKickerSpeed(speed);
  }

  // This will set rollers to the exact same speeds (Natural Backspin)
  public void setTargetSpeed(double rpm) {
    this.targetTopRPM = rpm;
    this.targetBottomRPM = rpm;
  }

  // Sets different speeds for Top and Bottom Rollers
  // This is useful  for controlling spin (backspin/topspin)
  public void setSplitSpeeds(double topRPM, double bottomRPM) {
    this.targetTopRPM = topRPM;
    this.targetBottomRPM = bottomRPM;
  }

  /**
   * @return True if flywheels are within tolerance of target
   */
  public boolean isAtSpeed() {
    boolean topReady = Math.abs(inputs.topRPM - targetTopRPM) < ShooterConstants.kFlywheelToleranceRPM;
    boolean bottomReady = Math.abs(inputs.bottomRPM - targetBottomRPM) < ShooterConstants.kFlywheelToleranceRPM;

    return topReady && bottomReady;
  }
}
