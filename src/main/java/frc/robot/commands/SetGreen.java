package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class SetGreen extends CommandBase{
    private LEDSubsystem s_lights;

    public SetGreen(LEDSubsystem lights){
        s_lights = lights;
    }

    @Override
    public void execute(){
        s_lights.setGreen();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
    

}
