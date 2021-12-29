package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class DriveClimber extends CommandBase {

    ClimberSubsystem climberSubsystem;
    XboxController opController = new XboxController(1);

    public DriveClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(opController.getY(GenericHID.Hand.kLeft) > 0.15) {
            climberSubsystem.setDog(true);
            climberSubsystem.setSpeed(-opController.getY(GenericHID.Hand.kLeft));
        }
        else if (opController.getY(GenericHID.Hand.kLeft) < -0.15) {
            climberSubsystem.setDog(false);
            climberSubsystem.setSpeed(-opController.getY(GenericHID.Hand.kLeft));
        } else {
            climberSubsystem.setSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
