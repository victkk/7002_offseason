package frc.robot.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakerConstants;
import frc.robot.Constants.PathfindConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intaker;
public class AutoCommandFactory {

    public static Command followPathIntakeAndShoot(DrivetrainSubsystem mDrivetrainSubsystem,Intaker mIntaker,Shooter mShooter,String Path1,String Path2){
        Optional<Alliance> currentAlliance = DriverStation.getAlliance();
                  PathPlannerPath startPath = PathPlannerPath.fromPathFile(Path1);
                  PathPlannerPath returnPath = PathPlannerPath.fromPathFile(Path2);
                  
                //   if (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red) {
                //     startPath = startPath.flipPath();
                //     returnPath = returnPath.flipPath();
                //   }
        
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(startPath),
                new IntakeCommand(mIntaker)
                ),
            new ParallelCommandGroup(
                AutoBuilder.followPath(returnPath),
                new ShootCommand(mShooter, ShooterConstants.SHOOT_RPS)
            ),
            new FeedCommand(mIntaker),
            new InstantCommand(()->{mShooter.stop();mIntaker.stop();}),
            new InstantCommand(()->mIntaker.setAngle(IntakerConstants.INTAKE_ANGLE)),
            new WaitCommand(0.5)
        );
    }


    public static Command zeroAndShootPreload(DrivetrainSubsystem mDrivetrainSubsystem,Intaker mIntaker,Shooter mShooter,String StartingPath){
        PathPlannerPath path;

        Optional<Alliance> currentAlliance = DriverStation.getAlliance();
                  
        if (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red) {
        path = PathPlannerPath.fromPathFile(StartingPath).flipPath();
        }
        else{
            path = PathPlannerPath.fromPathFile(StartingPath);
        }
        
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                    mDrivetrainSubsystem.runZeroingCommand(),
                    // if odom pose is close enough to the starting pose to 
                    Commands.either(AutoBuilder.pathfindToPoseFlipped(path.getPreviewStartingHolonomicPose(), PathfindConstants.constraints),
                    new InstantCommand(
                    () -> mDrivetrainSubsystem.setPose(path.getPreviewStartingHolonomicPose())),
                    ()->{return path.getPreviewStartingHolonomicPose().minus(mDrivetrainSubsystem.getPose()).getTranslation().getNorm()<1.0;}
                    )
                    ),
                
                new InstantCommand(()->mIntaker.setAngle(IntakerConstants.FEED_ANGLE+15.0)).andThen(new ShootCommand(mShooter, ShooterConstants.SHOOT_RPS)).
                    andThen(new FeedCommand(mIntaker)).andThen(new InstantCommand(()->{mShooter.stop();mIntaker.stop();}))
        );
    }

    public static Command zeroAndShootPreload2(DrivetrainSubsystem mDrivetrainSubsystem,Intaker mIntaker,Shooter mShooter,String StartingPath){
        PathPlannerPath path;

        Optional<Alliance> currentAlliance = DriverStation.getAlliance();
                  
        if (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red) {
        path = PathPlannerPath.fromPathFile(StartingPath).flipPath();
        }
        else{
            path = PathPlannerPath.fromPathFile(StartingPath);
        }
        
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                    mDrivetrainSubsystem.runZeroingCommand(),
                    // if odom pose is close enough to the starting pose to 
                    Commands.either(AutoBuilder.pathfindToPoseFlipped(path.getPreviewStartingHolonomicPose(), PathfindConstants.constraints),
                    new InstantCommand(
                    () -> mDrivetrainSubsystem.setPose(path.getPreviewStartingHolonomicPose())),
                    ()->{return path.getPreviewStartingHolonomicPose().minus(mDrivetrainSubsystem.getPose()).getTranslation().getNorm()<1.0;}
                    )
                    ),
                
                new InstantCommand(()->mIntaker.setAngle(IntakerConstants.FEED_ANGLE+15.0)).andThen(new InstantCommand(()->mShooter.setSpeed(15.0))).
                    andThen(new FeedCommand(mIntaker)).andThen(new InstantCommand(()->{mShooter.stop();mIntaker.stop();}))
        );
    }
}
