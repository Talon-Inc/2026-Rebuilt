package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
  private final SparkMax deployMotor;
  private final SparkMax intakeMotor;
  private final AbsoluteEncoder deployEncoder;
  private final SparkClosedLoopController deployController;

  public IntakeIOReal() {
    deployMotor = new SparkMax(IntakeConstants.kDeployMotorId, MotorType.kBrushless);
    intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
    deployEncoder = deployMotor.getAbsoluteEncoder();
    deployController = deployMotor.getClosedLoopController();

    // Apply some limits (These are only SOFT limits)
    SparkMaxConfig safedeployConfig = IntakeConfigs.deployConfig;
    safedeployConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(115.0) // Bottom limit
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(25.0); // Top Limit

    deployMotor.configure(
        safedeployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(
        IntakeConfigs.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.deployAngleDeg = deployEncoder.getPosition();
    inputs.deployVolts = deployMotor.getAppliedOutput() * deployMotor.getBusVoltage();
    inputs.deployAmps = new double[] {deployMotor.getOutputCurrent()};

    inputs.rollerRPM = intakeMotor.getEncoder().getVelocity();
    inputs.rollerVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.rollerAmps = new double[] {intakeMotor.getOutputCurrent()};
  }

  @Override
  public void setDeployPosition(double angleDeg, double feedForwardVolts) {
    // We'll test sending gravity voltage (Although we might not need this)
    deployController.setSetpoint(
        angleDeg,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedForwardVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setRollerVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void updatePID(double kP, double kI, double kD, double kS, double kV) {
    // This way it only applies new config when the dashboard numbers change
    SparkMaxConfig updateConfig = new SparkMaxConfig();
    updateConfig.closedLoop.p(kP).i(kI).d(kD).feedForward.kS(kS).kV(kV);
    deployMotor.configure(
        updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
