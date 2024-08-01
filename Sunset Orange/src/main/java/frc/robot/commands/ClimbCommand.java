package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import java.util.function.Supplier;

public class ClimbCommand extends Command{
    private final Climber mClimber;
    
    private final Supplier<Double> climbDutySupplier;
    
    public ClimbCommand(Climber climber,Supplier<Double> climbDutySupplier){
        mClimber = climber;
        this.climbDutySupplier = climbDutySupplier;
        addRequirements(mClimber);
    }
    @Override
    public void execute() {
        mClimber.setClimberDuty(climbDutySupplier.get());
    }
    @Override
    public void end(boolean interrupted){
        mClimber.stop();
    }
}
