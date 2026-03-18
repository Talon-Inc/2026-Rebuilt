// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShootingPhysics;
import frc.robot.util.ShootingPhysics.ShotType;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  private final Shooter shooter;
  private final Drive drive;
  private final Supplier<Translation2d> targetSupplier;
  private final ShootingPhysics.ShotType shotType;

  /** Creates a new ShootCommand. */
  public ShootCommand(Shooter shooter, Drive drive, Supplier<Translation2d> target, ShotType type) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.drive = drive;
    this.targetSupplier = target;
    this.shotType = type;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
    ShootingPhysics.ShootSolution solution;

    if (this.shotType == ShootingPhysics.ShotType.SCORE) {
      solution = ShootingPhysics.calculateShot(robotPose, fieldSpeeds, targetSupplier.get());
    } else {
      solution = ShootingPhysics.calculatePass(robotPose, fieldSpeeds, targetSupplier.get());
    }

    shooter.setSplitSpeeds(solution.topRPM(), solution.bottomRPM());

    // This is so it only feeds the ball when the flywheels are ready
    if (shooter.isAtSpeed()) {
      shooter.setKickerSpeed(0.5);
    } else {
      shooter.setKickerSpeed(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setTargetSpeed(0);
    shooter.setKickerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
