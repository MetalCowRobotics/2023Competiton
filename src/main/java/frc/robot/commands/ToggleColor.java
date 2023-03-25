package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class ToggleColor extends CommandBase{
    private LEDSubsystem s_lights;

    public ToggleColor(LEDSubsystem lights){
        s_lights = lights;
    }

    @Override
    public void execute(){
        if ((s_lights.getColor().equals("Default")) || (s_lights.getColor().equals("Purple"))){
            s_lights.setYellow();
        }
        else if ((s_lights.getColor().equals("Yellow"))){
            s_lights.setPurple();
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
    

    
}
