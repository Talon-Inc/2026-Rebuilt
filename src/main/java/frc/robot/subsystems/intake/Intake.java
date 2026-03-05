// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkMax m_intakeMotor1 = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
  private final SparkMax m_intakeMotor2 = new SparkMax(IntakeConstants.kMiddleIntakeCanId, MotorType.kBrushless);

  /** Creates a new Intake. */
   public Intake() {
  
   
  }

  public void intakefuel() {
    m_intakeMotor1.set(IntakeConstants.kSpeed);
   
  }

  

  public void outtakeNote() {
    m_intakeMotor1.set(-IntakeConstants.kSpeed);
  
  }

  public void stop() {
    m_intakeMotor1.stopMotor(); 
    m_intakeMotor2.stopMotor();
  }

  
  public boolean isIntakeRunning() {
   
    return Math.abs(m_intakeMotor1.getEncoder().getVelocity()) > 0.1;
  }

  @Override
  public void periodic() {
    
    
  }
}


  

