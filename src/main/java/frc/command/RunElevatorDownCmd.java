package frc.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorSubsystem;

//change so its automatic, currently it runs when the user holds it
public class RunElevatorDownCmd extends Command{
    private ElevatorSubsystem elevator;
    Timer timer = new Timer();
    double time = 15;
    
 
    public RunElevatorDownCmd(ElevatorSubsystem theelevator){
        this.elevator = theelevator;
        // addRequirements(elevator); 
    }

    public RunElevatorDownCmd(ElevatorSubsystem theelevator, double time){
        this.elevator = theelevator;
        // addRequirements(elevator); 
        this.time = time;
    }

    // @Override
    // public void initialize(){
    //     timer.start();
    // }

    @Override
    public void execute(){
        elevator.moveToBottom();
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
