package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import java.util.function.Supplier;

public class ClimbCommand extends Command{
    private final Climber mClimber;
    
    private final Supplier<Double> climbLeftDutySupplier,climbRightDutySupplier;
    
    public ClimbCommand(Climber climber,Supplier<Double> climbLeftDutySupplier,Supplier<Double> climbRightDutySupplier){
        mClimber = climber;
        this.climbLeftDutySupplier = climbLeftDutySupplier;
        this.climbRightDutySupplier = climbRightDutySupplier;
        
        addRequirements(mClimber);
    }
    @Override
    public void execute() {
        double left =0;
        double right = 0;
        if(Math.abs(climbLeftDutySupplier.get())<0.1){
            left = Math.abs(climbLeftDutySupplier.get())*climbLeftDutySupplier.get()*0.5;
        }
        if(Math.abs(climbRightDutySupplier.get())<0.1){
            right = Math.abs(climbRightDutySupplier.get())*climbRightDutySupplier.get()*0.5;
        }
        mClimber.setClimberDuty(left, right);
    }
    @Override
    public void end(boolean interrupted){
        mClimber.stop();
    }
}
