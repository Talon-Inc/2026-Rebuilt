// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
<<<<<<< HEAD
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
=======
import com.revrobotics.spark.SparkMax;
>>>>>>> 5bcba48bcbfb3f607df858e317035ecef4cc5476

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
<<<<<<< HEAD
    encoder = deployMotor.getAbsoluteEncoder();
=======

    deployMotor.configure(IntakeConfigs.deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(IntakeConfigs.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
>>>>>>> 5bcba48bcbfb3f607df858e317035ecef4cc5476
  }

  // used to lower and raise intake
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

  public boolean isIntakeRunning() {
    return Math.abs(deployMotor.getEncoder().getVelocity()) > 0.1;
  }

  @Override
  public void periodic() {
    if (encoder.getPosition() < 0 || encoder.getPosition() > 0.25)
      stopDeploy();
  }
}


  

