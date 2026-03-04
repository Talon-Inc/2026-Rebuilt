package frc.robot.subsystems.Intake;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase{
  private final SparkMax m_intakeMotor1 = new SparkMax(IntakeConstants.kIntakeCanId, IntakeConstants.kMotorType);
  private final SparkMax m_intakeMotor2 = new SparkMax(IntakeConstants.kMiddleIntakeCanId, IntakeConstants.kMotorType);
  private final AnalogInput m_intakeSensor = new AnalogInput(IntakeConstants.kSensorAnalogPort);



}