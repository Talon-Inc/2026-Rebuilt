// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ShootingPhysics;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveAimSOTF extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final Supplier<Translation2d> targetSupplier;
  private final DoubleSupplier xInput;
  private final DoubleSupplier yInput;

  private final ProfiledPIDController turnController;

  // Tunable Numbers
  private static final LoggedTunableNumber turnkP =
      new LoggedTunableNumber("Tuning/DriveAim/kP", 5.0);
  private static final LoggedTunableNumber turnKi =
      new LoggedTunableNumber("Tuning/DriveAim/kI", 0.0);
  private static final LoggedTunableNumber turnKd =
      new LoggedTunableNumber("Tuning/DriveAim/kD", 0.0);

  private static final LoggedTunableNumber speedLimit =
      new LoggedTunableNumber("Tuning/DriveAim/MaxSpeed", 4.5);

  // This limits the velocity of the driver (x & y)
  // Remember max MAGNITUDE is 4.8m/s
  // private static final InterpolatingDoubleTreeMap maxTranslationSpeedMap =
  //     new InterpolatingDoubleTreeMap();

  static {
    // Key: meters, Value: m/s
    // maxTranslationSpeedMap.put(null, null);
  }

  /** Creates a new DriveAimSOTF. */
  public DriveAimSOTF(
      Drive drive,
      Shooter shooter,
      Supplier<Translation2d> target,
      DoubleSupplier xInput,
      DoubleSupplier yInput) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drive = drive;
    this.shooter = shooter;
    this.targetSupplier = target;
    this.xInput = xInput;
    this.yInput = yInput;

    addRequirements(drive, shooter);

    turnController =
        new ProfiledPIDController(
            5.0,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(540), Units.degreesToRadians(720)));
    turnController.setPID(turnkP.get(), turnKi.get(), turnKd.get());
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Update the tunables
    if (turnkP.hasChanged() || turnKi.hasChanged() || turnKd.hasChanged()) {
      turnController.setPID(turnkP.get(), turnKi.get(), turnKd.get());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 1. Get Inputs
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());

    // 2. Calculate SOTF solution
    var solution = ShootingPhysics.calculateShot(robotPose, fieldSpeeds, targetSupplier.get());

    // 3. Calculate Turn Output
    double rotationOutput =
        turnController.calculate(
            robotPose.getRotation().getRadians(), solution.robotHeading().getRadians());

    // Read the driver's input
    double driverX = xInput.getAsDouble() * speedLimit.get();
    double driverY = yInput.getAsDouble() * speedLimit.get();

    // Get the distance between the HUB and robot
    double currentDistance = targetSupplier.get().getDistance(robotPose.getTranslation());

    // Get the speed limit
    double maxAllowedSpeed = 1; // maxTranslationSpeedMap.get(currentDistance);

    // Get where the dirver wants to go
    double requestedSpeed = Math.hypot(driverX, driverY);

    // Scale it down so the roobot can turn
    if (requestedSpeed > maxAllowedSpeed) {
      driverX = (driverX / requestedSpeed) * maxAllowedSpeed;
      driverY = (driverY / requestedSpeed) * maxAllowedSpeed;
    }

    // 4. Drive
    // Pass Driver X/y, but overridr rotation with SOTF result
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driverX, driverY, rotationOutput, drive.getRotation()));

    // 5. Set Shooter Speed
    shooter.setSplitSpeeds(solution.bottomRPM(), solution.topRPM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
