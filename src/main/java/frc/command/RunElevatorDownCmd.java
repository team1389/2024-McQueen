package frc.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorSubsystem;

//change so its automatic, currently it runs when the user holds it
public class RunElevatorDownCmd extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    Timer timer = new Timer();
    double time = 15;
    
 
    public RunElevatorDownCmd(ElevatorSubsystem theelevator){
        this.elevatorSubsystem = theelevator;
        // addRequirements(elevatorSubsystem); 
    }

    public RunElevatorDownCmd(ElevatorSubsystem theelevator, double time){
        this.elevatorSubsystem = theelevator;
        // addRequirements(elevatorSubsystem); 
        this.time = time;
    }

    // @Override
    // public void initialize(){
    //     timer.start();
    // }

    @Override
    public void execute(){
        elevatorSubsystem.moveToBottom();
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.stop();
    }
    // @Override
    // public boolean isFinished(){
    //     return timer.hasElapsed(time);
    // }

}
