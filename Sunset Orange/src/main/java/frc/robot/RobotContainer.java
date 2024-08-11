// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.auto.modes.*;
import frc.robot.commands.DriveWithTriggerCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SnapToAngleCommand;
import frc.robot.commands.SuckFromSourceCommand;
import frc.robot.commands.VisionShootCommand;
import frc.robot.lib6907.CommandSwerveController;
import frc.robot.lib6907.CommandSwerveController.DriveMode;
import frc.robot.subsystems.ApriltagCoprocessor;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Shooter;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private SendableChooser<Command> mChooser = new SendableChooser<>();

  // * Controllers */
  private final CommandSwerveController driverController = new CommandSwerveController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  /* Subsystems */
  private final DrivetrainSubsystem sDrivetrainSubsystem = new DrivetrainSubsystem();
  public final Intaker sIntaker = new Intaker();
  public final Shooter sShooter = new Shooter();

  /* pre-constructed commands */
  private final Command mZeroingCommand = sDrivetrainSubsystem.runZeroingCommand();

  private final SnapToAngleCommand mDriveWithRightStick = new SnapToAngleCommand(
      sDrivetrainSubsystem,
      () -> driverController.getDriveTranslation(driverController.isRobotRelative()),
      () -> driverController.getDriveRotationAngle(), // amp heading
      () -> driverController.isRobotRelative() == DriveMode.ROBOT_ORIENTED);
    
  private final SnapToAngleCommand mSnapToSource = new SnapToAngleCommand(
      sDrivetrainSubsystem,
      () -> driverController.getDriveTranslation(driverController.isRobotRelative()),
      () -> DriverStation.getAlliance().get()==Alliance.Blue?Optional.of(new Rotation2d(90.0)):Optional.of(new Rotation2d(90.0)), // amp heading
      () -> driverController.isRobotRelative() == DriveMode.ROBOT_ORIENTED);

  private final SnapToAngleCommand mSnapToAmp = new SnapToAngleCommand(
      sDrivetrainSubsystem,
      () -> driverController.getDriveTranslation(driverController.isRobotRelative()),
      () -> Optional.of(new Rotation2d(90.0)), // amp heading
      () -> driverController.isRobotRelative() == DriveMode.ROBOT_ORIENTED);

  private final DriveWithTriggerCommand mDriveWithTriggerCommand = new DriveWithTriggerCommand(sDrivetrainSubsystem,
      () -> driverController.getDriveTranslation(driverController.isRobotRelative()),
      () -> driverController.getRawRotationRate(), // amp heading
      () -> driverController.isRobotRelative() == DriveMode.ROBOT_ORIENTED);
  
  private final VisionShootCommand mVisionShootCommand  =new VisionShootCommand(sDrivetrainSubsystem,sShooter,sIntaker,() -> driverController.getDriveTranslation(driverController.isRobotRelative()) , () -> driverController.isRobotRelative() == DriveMode.ROBOT_ORIENTED);
      // private final ClimbCommand mClimbCommand = new ClimbCommand(sClimber, ()->{if(driverController.getRightX()>0.2)return driverController.getRightX()/5.0;else return 0.0;});
  private final IntakeCommand mIntakeCommand = new IntakeCommand(sIntaker);
  private final FeedCommand mFeedCommand = new FeedCommand(sIntaker);
  private final ShootCommand mShootCommand = new ShootCommand(sShooter,ShooterConstants.SHOOT_RPS);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    sDrivetrainSubsystem.setDefaultCommand(mDriveWithRightStick);
    configureBindings();
    sDrivetrainSubsystem.configureAutoBuilder();
    pushChooser();
    SmartDashboard.putData(sDrivetrainSubsystem);
    SmartDashboard.putData(sIntaker);
    SmartDashboard.putData(mDriveWithTriggerCommand);
    SmartDashboard.putData(sShooter);

    ApriltagCoprocessor.getInstance().setLoggingEnabled(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command resetHeadingCommand = new InstantCommand(
        () -> {
          sDrivetrainSubsystem.zeroHeading();
          driverController.setTranslationDirection(true);
        });
    
    resetHeadingCommand.addRequirements(sDrivetrainSubsystem);
    driverController.start().onTrue(resetHeadingCommand);
    new Trigger(() -> driverController.getRawRotationRate() != 0.0).whileTrue(mDriveWithTriggerCommand);
    operatorController.a().whileTrue(mIntakeCommand);
    operatorController.x().whileTrue(mShootCommand.andThen(mFeedCommand)).onFalse(new InstantCommand(()->{sShooter.stop();sIntaker.stop();}));
    operatorController.b().whileTrue(new SuckFromSourceCommand(sShooter,sIntaker));
    operatorController.y().whileTrue(new InstantCommand(()->sIntaker.setAngle(IntakerConstants.AMP_ANGLE)).andThen(new WaitCommand(1.0)).andThen(new InstantCommand(()->sIntaker.setRollerAmp()))).onFalse(new InstantCommand(()->sIntaker.stop()));
    
    driverController.x().whileTrue(mVisionShootCommand);
    driverController.a().whileTrue(mSnapToAmp);
    driverController.b().whileTrue(mSnapToSource.alongWith(new SuckFromSourceCommand(sShooter, sIntaker)));
      }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return mChooser.getSelected();
  }

  public void checkDrivetrainZeroing() {
    mZeroingCommand.schedule();
  }

  public void pushChooser() {
    // init points
    mChooser = new SendableChooser<>();
    mChooser.addOption("33", new mid33AutoCommand(sDrivetrainSubsystem, sIntaker, sShooter));

    SmartDashboard.putData("AUTO CHOICES", mChooser);
  }
}
