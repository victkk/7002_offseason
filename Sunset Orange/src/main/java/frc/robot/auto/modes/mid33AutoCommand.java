package frc.robot.auto.modes;

import frc.robot.subsystems.Intaker;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter;
public class mid33AutoCommand extends SequentialCommandGroup{
    public mid33AutoCommand(DrivetrainSubsystem mDrivetrainSubsystem,Intaker mIntaker, Shooter mShooter){
        addCommands(AutoCommandFactory.zeroAndShootPreload(mDrivetrainSubsystem, mIntaker, mShooter));
        addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"homeTo31","31ToHome"));
        addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"homeTo32","32ToHome"));
        addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"homeTo33","33ToHome"));
        addRequirements(mDrivetrainSubsystem,mIntaker,mShooter);
    }
}
