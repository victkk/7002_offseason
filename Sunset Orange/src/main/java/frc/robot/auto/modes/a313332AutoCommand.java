package frc.robot.auto.modes;
import frc.robot.subsystems.Intaker;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter;


public class a313332AutoCommand extends SequentialCommandGroup{
    public a313332AutoCommand(DrivetrainSubsystem mDrivetrainSubsystem,Intaker mIntaker, Shooter mShooter){
        addCommands(AutoCommandFactory.zeroAndShootPreload(mDrivetrainSubsystem, mIntaker, mShooter,"homeTo31"));
        addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"homeTo31","31ToHome"));
        addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"homeTo33","33ToHome"));
        addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"homeTo32","32ToHome"));
        addRequirements(mDrivetrainSubsystem,mIntaker,mShooter);
    }
}
