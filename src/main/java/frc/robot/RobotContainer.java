// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveAimSOTF;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ShootingPhysics.ShotType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  @SuppressWarnings("unused")
  private final Vision vision;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems
    intake = new Intake(new IntakeIOReal());
    shooter = new Shooter(new ShooterIOReal());

    drive =
        new Drive(
            new GyroIONavX(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));

    vision =
        new Vision(
            drive::addVisionMeasurement, new VisionIOPhotonVision(camera0Name, robotToCamera0));
    // new VisionIOPhotonVision(camera1Name, robotToCamera1));

    // switch (Constants.currentMode) {
    //   case REAL:
    //     // Real robot, instantiate hardware IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIONavX(),
    //             new ModuleIOSpark(0),
    //             new ModuleIOSpark(1),
    //             new ModuleIOSpark(2),
    //             new ModuleIOSpark(3));
    //     break;

    //   case SIM:
    //     // Sim robot, instantiate physics sim IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIO() {},
    //             new ModuleIOSim(),
    //             new ModuleIOSim(),
    //             new ModuleIOSim(),
    //             new ModuleIOSim());
    //     break;

    //   default:
    //     // replayed robot, disable IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {});
    //     break;
    // }

    // Setup Commands for Pathplanner
    NamedCommands.registerCommand(
        "Intake", Commands.runOnce(() -> intake.setState(IntakeState.INTAKE)));
    NamedCommands.registerCommand(
        "Stow", Commands.runOnce(() -> intake.setState(IntakeState.STOW)));
    NamedCommands.registerCommand(
        "AutoShoot",
        new ShootCommand(
            shooter,
            drive,
            intake,
            () -> AllianceFlipUtil.apply(Constants.FieldConstants.Goals.blueHub),
            ShotType.SCORE));
    NamedCommands.registerCommand(
        "AlignRotation",
        new DriveAimSOTF(
            drive,
            () -> AllianceFlipUtil.apply(Constants.FieldConstants.Goals.blueHub),
            () -> -controller.getLeftX(),
            () -> -controller.getLeftY(),
            ShotType.SCORE));
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  // Selects whether to pass to the left or right gap based on robot's Y position
  private Translation2d getSmartPassTarget() {
    Pose2d currentPose = drive.getPose();
    double fieldCenterY = Constants.FieldConstants.fieldWidth / 2.0;

    Translation2d baseTarget;

    if (currentPose.getY() > fieldCenterY) {
      baseTarget = Constants.FieldConstants.bluePassLeft;
    } else {
      baseTarget = Constants.FieldConstants.bluePassRight;
    }

    return AllianceFlipUtil.apply(baseTarget);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .square()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.cross().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .circle()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // controller
    //     .y()
    //     .toggleOnTrue(
    //         Commands.run(
    //             () ->
    //                 intake.runIntake(1))
    //         Commands.stop(
    //             () ->
    //                 intake.stopIntake()));
    // Shooter Buttons
    // controller.rightTrigger(.5).whileTrue(shoot);

    // --INTAKE--
    // Hold L1 to intake and when you release it automatically Stows
    // controller
    //     .povDown()
    //     .onTrue(Commands.runOnce(() -> intake.setState(IntakeState.INTAKE)))
    //     .onFalse(Commands.runOnce(() -> intake.setState(IntakeState.STOW)));
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (intake.getState() == IntakeState.INTAKE) {
                    intake.setState(IntakeState.STOW);
                  } else {
                    intake.setState(IntakeState.INTAKE);
                  }
                }));

    // -- DEFAULT Scoring Buttons --

    // Left Trigger: Aim at Hub
    controller
        .L2()
        .whileTrue(
            new DriveAimSOTF(
                drive,
                () -> AllianceFlipUtil.apply(Constants.FieldConstants.Goals.blueHub),
                () -> -controller.getLeftX(),
                () -> -controller.getLeftY(),
                ShotType.SCORE));

    // Right Bumper: Fire to Score
    controller
        .R2()
        .whileTrue(
            new ShootCommand(
                shooter,
                drive,
                intake,
                () -> AllianceFlipUtil.apply(Constants.FieldConstants.Goals.blueHub),
                ShotType.SCORE));

    // -- Passing Shots --

    // Left Trigger + POV Left (Aim to the closest gap)
    controller
        .L2()
        .and(controller.povLeft())
        .whileTrue(
            new DriveAimSOTF(
                drive,
                () -> getSmartPassTarget(),
                () -> -controller.getLeftX(),
                () -> -controller.getLeftY(),
                ShotType.PASS));

    // Right Bumper + POV Left (Fire to Pass)
    controller
        .R2()
        .and(controller.povLeft())
        .whileTrue(
            new ShootCommand(shooter, drive, intake, () -> getSmartPassTarget(), ShotType.PASS));

    controller.R1().whileTrue(new Shoot(shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
