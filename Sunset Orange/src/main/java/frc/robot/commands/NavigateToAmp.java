package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FiendConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.utils.ShootingParameters;

public class NavigateToAmp extends SequentialCommandGroup {
        /*
         * Implements navigate to shoot,
         * driver only need one button and one button only
         * to drive the robot back to shooting pos and start shooting
         */

        public NavigateToAmp(DrivetrainSubsystem drivetrain, Shooter shooter, Transfer transfer, Arm arm,
                        boolean isRedAlliance) {
                Pose2d shootingPose = FiendConstants.IN_FRONT_AMP_POSITION_BLUE;
                if (isRedAlliance) {
                        shootingPose = FiendConstants.IN_FRONT_AMP_POSITION_RED;
                }

                // Create path constraints
                PathConstraints constraints = new PathConstraints(3.0, 2.0, Units.degreesToRadians(540),
                                Units.degreesToRadians(720));

                // Create path finding command
                Command pathfindingCommand = AutoBuilder.pathfindToPose(shootingPose, constraints, 0, 0.0);

                // Create the shoot command
                Command shootCommand = new SetShooterTargetCommand(shooter, ShootingParameters.AMP.speed_rps)
                                .alongWith(new SetArmAngleCommand(arm, ShootingParameters.AMP.angle_deg))
                                .andThen(new FeedCommand(transfer));

                // Create the stop shooting command
                Command stopShootingCommand = new InstantCommand(shooter::stop)
                                .andThen(new SetArmAngleCommand(arm, ArmConstants.ARM_REST_ANGLE))
                                .andThen(new InstantCommand(() -> arm.stop()));

                // Add logging and SmartDashboard updates
                Command logPathfindingStarted = new InstantCommand(
                                () -> {
                                        System.out.println("NavigateToShoot: Pathfinding started");
                                        SmartDashboard.putString("NavigateToAmp/Status", "Pathfinding");
                                });

                Command logPathfindingFinished = new InstantCommand(
                                () -> {
                                        System.out.println("NavigateToAmp: Pathfinding finished");
                                        SmartDashboard.putString("NavigateToAmp/Status", "Shooting");
                                });

                Command logShootingFinished = new InstantCommand(
                                () -> {
                                        System.out.println("NavigateToAmp: Shooting finished");
                                        SmartDashboard.putString("NavigateToAmp/Status", "Finished");
                                });
                Command logCurrentAlliance = new InstantCommand(
                                () -> {
                                        SmartDashboard.putString("NavigateToAmp/Status", "Red Alliance");
                                });

                // Add the commands to the sequence
                addCommands(
                                logCurrentAlliance,
                                logPathfindingStarted,
                                pathfindingCommand,
                                logPathfindingFinished,
                                shootCommand,
                                stopShootingCommand,
                                logShootingFinished);
        }
}