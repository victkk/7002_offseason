package frc.robot.auto.modes;
import frc.robot.subsystems.Intaker;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter;


public class A515355AutoCommand extends SequentialCommandGroup{
    public A515355AutoCommand(DrivetrainSubsystem mDrivetrainSubsystem,Intaker mIntaker, Shooter mShooter){
        addCommands(AutoCommandFactory.zeroAndShootPreload(mDrivetrainSubsystem, mIntaker, mShooter,"HT51"));
        addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"HT51","51TH"));
       //// addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"HT53","53TH"));
      //  addCommands(AutoCommandFactory.followPathIntakeAndShoot(mDrivetrainSubsystem, mIntaker, mShooter,"HT55","55TH"));
        addRequirements(mDrivetrainSubsystem,mIntaker,mShooter);
    }
}
