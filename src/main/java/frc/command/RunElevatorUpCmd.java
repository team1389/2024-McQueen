package frc.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorSubsystem;

public class RunElevatorUpCmd extends Command{
    private ElevatorSubsystem elevator;
    Timer timer = new Timer();
    double time = 6.5;
    
    public RunElevatorUpCmd(ElevatorSubsystem elevator){
        this.elevator = elevator;
        // addRequirements(elevator); 
    }

    public RunElevatorUpCmd(ElevatorSubsystem elevator, double time){
        this.elevator = elevator;
        // addRequirements(elevator); 
        this.time = time;
    }

    // @Override
    // public void initialize(){
    //     timer.start();
    // }

    @Override
    public void execute(){
        elevator.moveToTop();
    }

    @Override
    public void end(boolean interrupted){
        elevator.stop();
    }
    // @Override
    // public boolean isFinished(){
    //     return timer.hasElapsed(time);
    // }

}
