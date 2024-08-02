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
    private static final double STABLIZE_TIME = 0.1;
    private static final double ERR_TOL = 5;
    private boolean isFinished = false;
    private DualEdgeDelayedBoolean spinStablized =
        new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(), STABLIZE_TIME, EdgeType.RISING);
    public ShootCommand(Shooter shooter,double target_rps){
        sShooter = shooter;
        this.target_rps = target_rps;
        addRequirements(sShooter);
    }

    @Override
    public void initialize() {
        sShooter.setSpeed(target_rps);
    }

    @Override
    public void execute(){
        double mainMotorVelocity = sShooter.getMainMotorVelocity();
        double followerVelocity = sShooter.getFollowerVelocity();
        SmartDashboard.putNumber("11",mainMotorVelocity);
        SmartDashboard.putNumber("22",followerVelocity);
        
        if (
                spinStablized.update(Timer.getFPGATimestamp(),Math.abs(mainMotorVelocity - target_rps) < ERR_TOL
                && Math.abs(followerVelocity - target_rps) < ERR_TOL))
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
