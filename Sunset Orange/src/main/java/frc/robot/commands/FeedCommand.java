package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakerConstants;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.subsystems.Intaker;

public class FeedCommand extends Command {
    Intaker sIntaker;
     private DualEdgeDelayedBoolean armStablized;
     private DualEdgeDelayedBoolean ringStablized;
    public FeedCommand(Intaker intaker){
        sIntaker = intaker;
        addRequirements(sIntaker);
    }
    @Override
    public void initialize() {
        sIntaker.setAngle(IntakerConstants.FEED_ANGLE);
        armStablized=new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(), 0.2, EdgeType.RISING);
        ringStablized=new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(), 0.04, EdgeType.RISING);
    }

    @Override
    public void execute(){
        if(armStablized.update(Timer.getFPGATimestamp(),Math.abs(sIntaker.getAngleDeg()-sIntaker.getTargetAngleDeg())<2))
            sIntaker.setRollerFeed();
    }
    @Override
    public void end(boolean isInterrupted){
        sIntaker.stopRoller();
    }
    @Override
    public boolean isFinished() {
        return ringStablized.update(Timer.getFPGATimestamp(),!sIntaker.isOmronDetected());
    }
}
