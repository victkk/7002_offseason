package frc.robot.auto.modes;
import frc.robot.subsystems.Intaker;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter;


public class mid33With51Fallback52 extends SequentialCommandGroup{
    public mid33With51Fallback52(DrivetrainSubsystem mDrivetrainSubsystem,Intaker mIntaker, Shooter mShooter){
        addCommands(AutoCommandFactory.zeroAndShootPreload(mDrivetrainSubsystem, mIntaker, mShooter,"homeTo31"));
        addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"homeTo31","31ToHome"));
        addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"homeTo32","32ToHome"));
        addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"homeTo33","33ToHome"));
        addCommands(AutoCommandFactory.followPathIntakeAndShootWithFallback(mDrivetrainSubsystem, mIntaker, mShooter,"homeTo51","51ToHome","51to52","52ToHome"));
        addRequirements(mDrivetrainSubsystem,mIntaker,mShooter);
    }
}
