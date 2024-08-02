package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakerConstants;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.subsystems.Intaker;

/**
 * @brief This command sets a single target (which must be determined at code initilization/before
 *     match)
 */
public class IntakeCommand extends Command {
  private final Intaker sIntaker;
  private boolean isFinished=false;
    
  public IntakeCommand(Intaker intaker) {
    sIntaker = intaker;
    addRequirements(sIntaker);
  }

  


  @Override
  public void initialize() {
    sIntaker.setAngle(170);
    sIntaker.setRollerIntake();
  }

  @Override
  public void execute() {

  }


  @Override
  public void end(boolean interrupted) {
    sIntaker.setAngle(30);
    sIntaker.stopRoller();
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
