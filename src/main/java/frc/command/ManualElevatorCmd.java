package frc.command;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorSubsystem;

public class ManualElevatorCmd extends Command{
    ElevatorSubsystem elevatorSubystem;
    private Supplier<Double> power;

    public ManualElevatorCmd(ElevatorSubsystem elevatorSubystem, Supplier<Double> power) {
        this.elevatorSubystem = elevatorSubystem;
        this.power = power;
        addRequirements(elevatorSubystem);
    }
    
    @Override
    public void execute() {
        elevatorSubystem.controllerInterrupt = true;
        elevatorSubystem.moveElevator(MathUtil.clamp(power.get(), -1, 1));
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubystem.moveElevator(0);
    }
}
