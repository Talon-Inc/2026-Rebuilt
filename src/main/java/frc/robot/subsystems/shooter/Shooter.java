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
  private final LoggedTunableNumber kRPM =
      new LoggedTunableNumber("/Tuning/Shooter/kRPM", ShooterConstants.kRPM);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("/Tuning/Shooter/kP", ShooterConstants.kP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("/Tuning/Shooter/kI", ShooterConstants.kI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("/Tuning/Shooter/kD", ShooterConstants.kD);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("/Tuning/Shooter/kS", ShooterConstants.kS);
  private final LoggedTunableNumber kVTop =
      new LoggedTunableNumber("/Tuning/Shooter/kV", ShooterConstants.kV[0]);
  private final LoggedTunableNumber kVBottom =
      new LoggedTunableNumber("/Tuning/Shooter/kV", ShooterConstants.kV[1]);
  private final LoggedTunableNumber kATop =
      new LoggedTunableNumber("/Tuning/Shooter/kA", ShooterConstants.kA[0]);
  private final LoggedTunableNumber kABottom =
      new LoggedTunableNumber("/Tuning/Shooter/kA", ShooterConstants.kA[1]);

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
    // Run Flywheel Logic
    runFlywheel(kRPM.get());

    io.configurePID(
        io.getTopMotor(),
        ShooterConfigs.topConfig,
        kP.get(),
        kI.get(),
        kD.get(),
        kS.get(),
        kVTop.get(),
        kATop.get());
    io.configurePID(
        io.getBottomMotor(),
        ShooterConfigs.bottomConfig,
        kP.get(),
        kI.get(),
        kD.get(),
        kS.get(),
        kVBottom.get(),
        kABottom.get());

    // Log Goals
    Logger.recordOutput("Shooter/Goal/PrimaryRPM", targetTopRPM);
    Logger.recordOutput("Shooter/Goal/SecondaryRPM", targetBottomRPM);
    Logger.recordOutput("Shooter/Goal/KickerRPM", targetKickerRPM);
  }

  // PID controlled velocity
  private void runFlywheel(double rpm) {
    io.setTopRPM(rpm);
    io.setBottomRPM(rpm);
  }

  // // Core Bang-Bang Logic
  // private void runFlywheel(
  //     BangBangController controller,
  //     double currentRPM,
  //     double target,
  //     boolean isTop,
  //     double voltageToUse) {
  //   // Only run if we actualy have a target (avoids jitter at 0 RPM)
  //   if (target > 50.0) {
  //     // .calculate() returns 1.0 if we need speed, 0.0 if we don't
  //     double demand = controller.calculate(currentRPM, target);

  //     // Multiply by Max Voltage (12V)
  //     double volts = demand * voltageToUse;

  //     if (isTop) io.setTopRPM(volts);
  //     else io.setBottomRPM(volts);
  //   } else {
  //     // Idle Mode
  //     if (isTop) io.setTopRPM(0.0);
  //     else io.setBottomRPM(0.0);
  //   }
  // }

  // --- Commands ---

  // This will set rollers to the exact same speeds (Natural Backspin)
  public void setTargetSpeed(double rpm) {
    this.targetTopRPM = rpm;
    this.targetBottomRPM = rpm;
  }

  // Sets different speeds for Top and Bottom Rollers
  // This is useful  for controlling spin (backspin/topspin)
  public void setSplitSpeeds(double topRPM, double bottomRPM) {
    io.setTopRPM(topRPM);
    io.setBottomRPM(bottomRPM);
    this.targetTopRPM = topRPM;
    this.targetBottomRPM = bottomRPM;
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

  public void setMotorRPM(double rpm) {
    io.setTopRPM(rpm);
    io.setBottomRPM(rpm);
  }
}
