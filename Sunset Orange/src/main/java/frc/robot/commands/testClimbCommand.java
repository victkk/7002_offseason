package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import java.util.function.Supplier;

public class testClimbCommand extends Command{
    private final Climber mClimber;
    
    private final Supplier<Double> climbDutySupplier;
    
    public testClimbCommand(Climber climber,Supplier<Double> climbDutySupplier){
        mClimber = climber;
        this.climbDutySupplier = climbDutySupplier;
        addRequirements(mClimber);
    }
    @Override
    public void execute() {
        if(Math.abs(climbDutySupplier.get())<0.1){
            mClimber.setClimberDuty(0);
        }else{
        mClimber.setClimberDuty(climbDutySupplier.get()*climbDutySupplier.get()*0.5);
        }
    }
    @Override
    public void end(boolean interrupted){
        mClimber.stop();
    }
}
