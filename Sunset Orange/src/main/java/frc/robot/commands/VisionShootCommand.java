package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionShootConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Shooter;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class VisionShootCommand extends Command {

  private final TrapezoidProfile.Constraints swerveRotateConstraints =
      new TrapezoidProfile.Constraints(Units.degreesToRadians(360), Units.degreesToRadians(540));
  private final ProfiledPIDController snapToAnglePID =
      new ProfiledPIDController(0.5, 0, 0.0, swerveRotateConstraints);
  // new ProfiledPIDController(4.0, 0, 0.2, swerveRotateConstraints);

  private final DrivetrainSubsystem mDrivetrainSubsystem;
  private final Shooter mShooter;
  private final Intaker mIntaker;

  private final Supplier<Translation2d> driveVectorSupplier;
  /*
   * Getting the angular velocity from the product of the joysticks (Right X) and the max angular velocity
   */
  private final BooleanSupplier robotCentricSupplier;
  private Rotation2d goalHeading;
  /**
   * The default drive command constructor
   *
   * @param drivetrainSubsystem The coordinator between the gyro and the swerve modules
   * @param xVelocitySupplier Gets the joystick value for the x velocity and multiplies it by the
   *     max velocity
   * @param yVelocitySupplier Gets the joystick value for the y velocity and multiplies it by the
   *     max velocity
   */
  public VisionShootCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Shooter shooter,
      Intaker intaker,
      Supplier<Translation2d> driveVectorSupplier,
      BooleanSupplier robotCentricSupplier) {
    this.setName("VisionShootCommand");
    mDrivetrainSubsystem = drivetrainSubsystem;
    mIntaker = intaker;
    mShooter = shooter;
    this.driveVectorSupplier = driveVectorSupplier;
    this.robotCentricSupplier = robotCentricSupplier;
    goalHeading = new Rotation2d(getGoalToRobot().getX(),getGoalToRobot().getY());
    addRequirements(drivetrainSubsystem); // required for default command
  }

  
  @Override
  public void initialize() {
    snapToAnglePID.reset(mDrivetrainSubsystem.getHeading().getRadians());
    snapToAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    snapToAnglePID.setGoal(mDrivetrainSubsystem.getHeading().getRadians());
    snapToAnglePID.setTolerance(0.05);
    
  }

  @Override
  public void execute() {
    // Running the lambda statements and getting the velocity values

    mShooter.setMaxVoltage();
    Translation2d driveVector =
        driveVectorSupplier
            .get()
            .times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond); // -1~1 to meters per second
    goalHeading = new Rotation2d(getGoalToRobot().getX(),getGoalToRobot().getY());
    snapToAnglePID.setGoal(goalHeading.getRadians());
    mDrivetrainSubsystem.drive(
        driveVector,
        snapToAnglePID.atGoal()
            ? 0
            : snapToAnglePID.calculate(mDrivetrainSubsystem.getHeading().getRadians())
                + snapToAnglePID.getSetpoint().velocity, // output is in radians per second
        !robotCentricSupplier.getAsBoolean());

        if(isAligned()&& mShooter.getMainMotorVelocity() - ShooterConstants.VISION_RPS >0
        && mShooter.getFollowerVelocity()- ShooterConstants.VISION_RPS > 0){
            mIntaker.setRollerFeed();
        }
  }

  @Override
  public void end(boolean interrupted) {
    mDrivetrainSubsystem.drive(new Translation2d(0, 0), 0, true);
    mShooter.stop();
    mIntaker.setAngle(IntakerConstants.FEED_ANGLE);
    mIntaker.stopRoller();
}

  @Override
  public boolean isFinished() {
    return false;
    // return isAligned();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "target heading error", () -> snapToAnglePID.getPositionError(), null);
 }

  public boolean isAligned() {

    double headingError =
        Rotation2d.fromRadians(snapToAnglePID.getGoal().position)
            .minus(goalHeading)
            .getDegrees();
    SmartDashboard.putNumber("heading error", headingError);
    return Math.abs(headingError) < 2.0;
  }
    /**
   * Calculates the Goal Position relative to robot, in field's coordinate system. If drivetrain is
   * moving, then we need to offset the goal position by a position vector which is time of note fly
   * times the robot's velocity
   *
   * @return goal position relative to robot, in field's coordinate system, unit is meter.
   */
  private Translation2d getGoalToRobot() {
    Translation2d robotToField = mDrivetrainSubsystem.getPose().getTranslation();
    Optional<Alliance> a = DriverStation.getAlliance();
    if (a.isEmpty()) {
      a=Optional.of(Alliance.Blue);
    }

    Translation2d goalToField =
        a.get() == Alliance.Red
            ? VisionShootConstants.kRedSpeaker
            : VisionShootConstants.kBlueSpeaker;

    Translation2d goalToRobot = goalToField.minus(robotToField);
    return goalToRobot;
  }
}
