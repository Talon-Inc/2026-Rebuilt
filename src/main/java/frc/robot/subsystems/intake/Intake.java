// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkMax deployMotor;
  private final SparkMax intakeMotor;
  private final AbsoluteEncoder encoder;

  /** Creates a new Intake. */
  public Intake() {
    deployMotor = new SparkMax(IntakeConstants.kDeployMotorId, MotorType.kBrushless);
    intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
    encoder = deployMotor.getAbsoluteEncoder();

    deployMotor.configure(IntakeConfigs.deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(IntakeConfigs.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // used to lower and raise intake
  // "+" lowers and "-" raises
  public void deploy(double speed) {
    deployMotor.set(speed);
  }

  // used to run the intake rollers
  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void stopDeploy() {
    deployMotor.stopMotor();
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public double getPosition(){
    return encoder.getPosition();
  }

  public boolean isIntakeRunning() {
    return Math.abs(deployMotor.getEncoder().getVelocity()) > 0.1;
  }

  @Override
  public void periodic() {
    double currentPos = encoder.getPosition(); // in degrees
    if (currentPos < 0 || currentPos > 90)
      stopDeploy();
  }
}


  

