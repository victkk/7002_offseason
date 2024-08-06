package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakerConstants;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.subsystems.Intaker;

public class FeedCommand extends Command {
    Intaker sIntaker;
     private DualEdgeDelayedBoolean spinStablized;
    public FeedCommand(Intaker intaker){
        sIntaker = intaker;
        addRequirements(sIntaker);
    }
    @Override
    public void initialize() {
        sIntaker.setAngle(IntakerConstants.FEED_ANGLE);
        spinStablized=new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(), 0.06, EdgeType.RISING);
    }

    @Override
    public void execute(){
        if(spinStablized.update(Timer.getFPGATimestamp(),Math.abs(sIntaker.getAngleDeg()-sIntaker.getTargetAngleDeg())<2))
            sIntaker.setRollerFeed();
    }
    @Override
    public void end(boolean isInterrupted){
        sIntaker.stopRoller();
    }
    
}
