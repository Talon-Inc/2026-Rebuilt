// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkMax deployMotor;
  private final SparkMax intakeMotor;

  /** Creates a new Intake. */
  public Intake() {
    deployMotor = new SparkMax(IntakeConstants.kDeployMotorId, MotorType.kBrushless);
    intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
  }

  // used to lower and raise intake
  public void deploy(double speed) {
    deployMotor.set(speed);
  }

  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void stop() {
    deployMotor.stopMotor();
    intakeMotor.stopMotor();
  }

  public boolean isIntakeRunning() {
    return Math.abs(deployMotor.getEncoder().getVelocity()) > 0.1;
  }

  @Override
  public void periodic() {}
}


  

