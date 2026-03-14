// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkMax deployMotor;
  private final SparkMax intakeMotor;
  private final AbsoluteEncoder encoder;
  private double lowerLimit;
  private double upperLimit;

  /** Creates a new Intake. */
  public Intake() {
    deployMotor = new SparkMax(IntakeConstants.kDeployMotorId, MotorType.kBrushless);
    intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
    encoder = deployMotor.getAbsoluteEncoder();

    deployMotor.configure(
        IntakeConfigs.deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(
        IntakeConfigs.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    lowerLimit = 0;
    upperLimit = 90;
  }

  // used to lower and raise intake
  // "+" lowers and "-" raises
  public void deploy(double speed) {
    deployMotor.set(speed);
  }

  public Command deployCommand(double speed) {
    return runOnce(() -> deploy(speed));
  }

  // used to run the intake rollers
  // "+" pulls balls in and "-" pushes them out
  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void stopDeploy() {
    deployMotor.stopMotor();
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  // Get angle of the deploy arm
  public double getPosition() {
    return encoder.getPosition();
  }

  // Check if intake motor is running
  public boolean isIntakeRunning() {
    return Math.abs(intakeMotor.getEncoder().getVelocity()) > 0.1;
  }

  // Set minimum angle for deploy
  public void setLowerLimit(double limit) {
    this.lowerLimit = limit;
  }

  // Set maximum angle for deploy
  public void setUpperLimit(double limit) {
    this.upperLimit = limit;
  }

  @Override
  public void periodic() {
    double currentPos = encoder.getPosition(); // in degrees
    if (currentPos < lowerLimit || currentPos > upperLimit) {
      stopDeploy();
    }
  }
}
