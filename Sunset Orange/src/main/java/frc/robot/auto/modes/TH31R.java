package frc.robot.auto.modes;
import frc.robot.subsystems.Intaker;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter;


public class TH31R extends SequentialCommandGroup{
    public TH31R(DrivetrainSubsystem mDrivetrainSubsystem,Intaker mIntaker, Shooter mShooter){
        addCommands(AutoCommandFactory.zeroAndShootPreload(mDrivetrainSubsystem, mIntaker, mShooter,"HT31"));
        addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"HT31","31TR"));
       // addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"HT33","33TH"));
        //addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"HT32","32TH"));
        addRequirements(mDrivetrainSubsystem,mIntaker,mShooter);
    }
}
