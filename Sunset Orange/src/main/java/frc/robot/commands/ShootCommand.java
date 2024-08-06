package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootCommand extends Command {
    Shooter sShooter;
    double target_rps;
    // private static final double STABLIZE_TIME = 0.1;

    private boolean isFinished = false;
    // private DualEdgeDelayedBoolean spinStablized =
    //     new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(), STABLIZE_TIME, EdgeType.RISING);
    public ShootCommand(Shooter shooter,double target_rps){
        sShooter = shooter;
        this.target_rps = target_rps;
        addRequirements(sShooter);
    }

    @Override
    public void initialize() {
        isFinished = false;
        sShooter.setMaxVoltage();
        // spinStablized = new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(),STABLIZE_TIME,EdgeType.RISING);
    }

    @Override
    public void execute(){
        double mainMotorVelocity = sShooter.getMainMotorVelocity();
        double followerVelocity = sShooter.getFollowerVelocity();

        
        if (
                mainMotorVelocity - target_rps >0
                && followerVelocity - target_rps > 0)
                isFinished= true;
        
    }
    @Override
    public void end(boolean isInterrupted){
    }
    @Override
    public boolean isFinished() {
      return isFinished;
    }
}
