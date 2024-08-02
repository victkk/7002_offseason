package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intaker;

public class FeedCommand extends Command {
    Intaker sIntaker;
    public FeedCommand(Intaker intaker){
        sIntaker = intaker;
        addRequirements(sIntaker);
    }
    @Override
    public void initialize() {
        sIntaker.setAngle(30);
        sIntaker.setRollerFeed();
    }

    @Override
    public void execute(){

    }
    @Override
    public void end(boolean isInterrupted){
        sIntaker.setAngle(30);
        sIntaker.stopRoller();

    }
    
}
