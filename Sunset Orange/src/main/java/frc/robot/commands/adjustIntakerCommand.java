package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakerConstants;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.subsystems.Intaker;
public class adjustIntakerCommand extends Command{


/**
 * @brief This command sets a single target (which must be determined at code initilization/before
 *     match)
 */
  private final Intaker sIntaker;
  private enum State{
      OUT,
      IN,
      END
    }

  State currentState;
  double timeStamp;
  boolean allowAdjust;
  public adjustIntakerCommand(Intaker intaker) {
    sIntaker = intaker;
    allowAdjust=false;
    addRequirements(sIntaker);
  }

  


  @Override
  public void initialize() {
    currentState = State.OUT;
    sIntaker.setAngle(IntakerConstants.ADJUST_ANGLE);
  }

  @Override
  public void execute() {
    if(sIntaker.getAngleDeg()-sIntaker.getTargetAngleDeg()<5.0){
      allowAdjust=true;
    }
    if(allowAdjust){
    if((!sIntaker.isOmronDetected())&&(currentState==State.OUT)){
      currentState = State.IN;
      timeStamp = Timer.getFPGATimestamp();
    }
    if(currentState==State.IN&&sIntaker.isOmronDetected()){
      currentState=State.END;
      timeStamp=Timer.getFPGATimestamp();
    }
    switch (currentState){
      case OUT:
        sIntaker.setRollerVoltage(-3.0);
        break;
      case IN:
        sIntaker.setRollerVoltage(3.0);
        break;
      case END:
        
        break;

    }
  }
        

  }


  @Override
  public void end(boolean interrupted) {
    sIntaker.setAngle(IntakerConstants.FEED_ANGLE+15.0);
    sIntaker.stopRoller();
  }

  @Override
  public boolean isFinished() {
    
    return currentState==State.END&&Timer.getFPGATimestamp()-timeStamp>0.03;
  }
}
