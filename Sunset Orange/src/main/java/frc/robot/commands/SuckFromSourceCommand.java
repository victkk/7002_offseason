package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakerConstants;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Shooter;

public class SuckFromSourceCommand extends Command{
    private Shooter sShooter;
    private Intaker sIntaker;
    public SuckFromSourceCommand(Shooter shooter,Intaker intaker){
        sShooter = shooter;
        sIntaker = intaker;
        addRequirements(intaker);
        addRequirements(shooter);
}
    @Override
    public void initialize() {
        sShooter.setSpeed(-20);
        sIntaker.setAngle(IntakerConstants.REST_ANGLE+1);
        sIntaker.setRollerIntake();
    }
    @Override
    public void end(boolean interrupted) {
        sShooter.stop();
        sIntaker.setAngle(IntakerConstants.FEED_ANGLE);
        sIntaker.stopRoller();    
    }

    @Override
    public boolean isFinished() {
        return sIntaker.isOmronDetected();
    }

}
