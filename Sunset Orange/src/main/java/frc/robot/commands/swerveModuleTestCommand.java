package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class swerveModuleTestCommand extends Command {

      private final DrivetrainSubsystem mDrivetrainSubsystem;

  public swerveModuleTestCommand(DrivetrainSubsystem drivetrainSubsystem,int moduleID) {
    mDrivetrainSubsystem = drivetrainSubsystem;
    addRequirements(mDrivetrainSubsystem);

  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
