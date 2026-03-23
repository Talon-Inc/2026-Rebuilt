// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // STATE MACHINE (Yes, I added this, it improves so much, I'll tell in person)
  public enum IntakeState {
    STOW, // Up, Roller is off
    INTAKE, // Down, Spinning in
    EJECT, // Down, Spinning out
    AGITATE, // Smart Agittation
    PREP // This will wait in position, rollers off (Sepcifically for ShootComand)
  }

  private IntakeState currentState = IntakeState.STOW;

  // Agitating memory
  private boolean agitateMovingDown = true;
  private double currentTargetAngle = 26.0; // This tracks where the arm is currently trying to go

  // Something I got from 6328: This waits .1 seconds to ensure the arm has stopped bouncing
  private final Debouncer atGoalDebouncer = new Debouncer(0.1, DebounceType.kRising);
  private boolean isStableAtGoal = false;

  // So if we hit an obstruction when trying to Stow it doesn't jitter
  private boolean hitObstruction = false;
  private double restingAngle = 26.0;

  private int agitateCooldownLoops = 0;
  private int stowCooldownLoops = 0;

  // Tuanable Numbers (For Deployment)
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("/Tuning/Intake/kP", IntakeConstants.kP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("/Tuning/Intake/kI", IntakeConstants.kI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("/Tuning/Intake/kD", IntakeConstants.kD);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("/Tuning/Intake/kS", IntakeConstants.kS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("/Tuning/Intake/kV", IntakeConstants.kV);
  private final LoggedTunableNumber kG =
      new LoggedTunableNumber("/Tuning/Intake/kG", IntakeConstants.kG);

  // Tunable Numbers (For Roller)
  private final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("/Tuning/Intake/RollerIntakeVolts", 10.5);
  private final LoggedTunableNumber feedVolts =
      new LoggedTunableNumber("/Tuning/Intake/RollerFeedVolts", 6.0);
  private final LoggedTunableNumber ejectVolts =
      new LoggedTunableNumber("/Tuning/Intake/RollerEjectVolts", -6.0);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setState(IntakeState state) {
    // If we are leaving stow, forget the obstruction so it can move again freely
    if (state != IntakeState.STOW) {
      hitObstruction = false;
    } else {
      stowCooldownLoops = 15;
    }
    this.currentState = state;
  }

  public IntakeState getState() {
    return currentState;
  }

  // Checks if the arm is done moving
  public boolean isStable() {
    return isStableAtGoal;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/State", currentState.toString());

    // Update PID only if we change it in Advantage Scope
    if (kP.hasChanged()
        || kI.hasChanged()
        || kD.hasChanged()
        || kS.hasChanged()
        || kV.hasChanged()
        || kG.hasChanged()) {
      io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get());
    }

    // Calculate the voltage for Gravity (kG * sin(angle))
    double effectiveAngle = inputs.deployAngleDeg - 26.0;
    double gravityVoltage = kG.get() * Math.sin(Math.toRadians(effectiveAngle));

    // Run the STATE MACHINE
    switch (currentState) {
      case STOW:
        double stowAmps = inputs.deployAmps.length > 0 ? inputs.deployAmps[0] : 0.0;

        // if we hit a spike for the first time, hold position
        if (stowCooldownLoops > 0) {
          stowCooldownLoops--;
        } else if (stowAmps > 60.0 && !hitObstruction) {
          hitObstruction = true;
          restingAngle = inputs.deployAngleDeg;
        }

        // if obstructed, hold the resting angle forever. if not then go to 3.0
        currentTargetAngle = hitObstruction ? restingAngle : 27.0;
        io.setDeployPosition(currentTargetAngle, gravityVoltage);
        io.setRollerVoltage(0.0);
        break;

      case PREP:
        currentTargetAngle = 45.0;
        io.setDeployPosition(currentTargetAngle, gravityVoltage);
        io.setRollerVoltage(0.0);
        break;

      case INTAKE:
        currentTargetAngle = 113.0;
        io.setDeployPosition(currentTargetAngle, gravityVoltage);
        io.setRollerVoltage(intakeVolts.get());
        break;

      case EJECT:
        currentTargetAngle = 113.0;
        io.setDeployPosition(currentTargetAngle, gravityVoltage);
        io.setRollerVoltage(ejectVolts.get());
        break;

      case AGITATE:
        // Keep rollers feeding the kicker (we can change this if it causes problems)
        io.setRollerVoltage(feedVolts.get());

        // By readinng the current draw of the motor we get a feel for resistance
        double currentAmps = inputs.deployAmps.length > 0 ? inputs.deployAmps[0] : 0.0;

        // If we hit a jam (>20A [THIS CAN CHANGE]), then we intstantly reverse direction
        if (agitateCooldownLoops > 0) {
          agitateCooldownLoops--;
        } else if (currentAmps > 60.0) {
          agitateMovingDown = !agitateMovingDown;
          agitateCooldownLoops = 15;
        }

        // LIMITS: Reverse direction if we hit our upper or lower bounds
        if (inputs.deployAngleDeg >= 48.0) {
          agitateMovingDown = false; // Go up
          agitateCooldownLoops = 15;
        } else if (!agitateMovingDown && inputs.deployAngleDeg <= 28.0) {
          agitateMovingDown = true; // Go down
          agitateCooldownLoops = 15;
        }

        // What makes it move
        currentTargetAngle = agitateMovingDown ? 45.0 : 26.0;
        io.setDeployPosition(currentTargetAngle, gravityVoltage);
        break;
    }

    Logger.recordOutput("Intake/TargetAngleDeg", currentTargetAngle);

    // Check if we are within 3 degrees of the target
    boolean atAngle = Math.abs(inputs.deployAngleDeg - currentTargetAngle) <= 3.0;
    isStableAtGoal = atGoalDebouncer.calculate(atAngle);
    Logger.recordOutput("Intake/IsStable", isStableAtGoal);
  }
}
