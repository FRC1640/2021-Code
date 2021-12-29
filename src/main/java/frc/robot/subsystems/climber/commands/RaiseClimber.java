package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class RaiseClimber extends CommandBase {
    
    ClimberSubsystem climberSubsystem;

    public RaiseClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climberSubsystem.setPistons(true);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}
