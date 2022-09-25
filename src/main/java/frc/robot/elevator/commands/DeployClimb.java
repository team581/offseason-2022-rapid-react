package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.ElevatorSubsystem;

public class DeployClimb extends CommandBase {
    private final ElevatorSubsystem elevator;
    public DeployClimb(ElevatorSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }
}
