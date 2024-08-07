package frc.robot.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
                  
                  if (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red) {
                    startPath = startPath.flipPath();
                    returnPath = returnPath.flipPath();
                  }
        
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(startPath),
                new IntakeCommand(mIntaker)
                ),
            new ParallelCommandGroup(
                AutoBuilder.followPath(returnPath),
                new ShootCommand(mShooter, ShooterConstants.SHOOT_RPS)
            ),
            new FeedCommand(mIntaker)
        );
    }


    public static Command zeroAndShootPreload(DrivetrainSubsystem mDrivetrainSubsystem,Intaker mIntaker,Shooter mShooter,String StartingPath){
        Optional<Alliance> currentAlliance = DriverStation.getAlliance();
        PathPlannerPath path = PathPlannerPath.fromPathFile(StartingPath);
        
        if (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red) {
        path = path.flipPath();
        }
        
        
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                    mDrivetrainSubsystem.runZeroingCommand(),
                    AutoBuilder.pathfindToPose(path.getPreviewStartingHolonomicPose(), PathfindConstants.constraints)
                ),
                new ShootCommand(mShooter, ShooterConstants.SHOOT_RPS).
                    andThen(new FeedCommand(mIntaker))
        );
    }
}
