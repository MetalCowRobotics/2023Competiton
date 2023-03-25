package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
    private Spark lightController;
    private String color;

    public LEDSubsystem(int portNum){
        lightController = new Spark(portNum);
        lightController.set(.93);
        color = "Default";
    }

    public void setYellow(){
        lightController.set(.65);
        color = "Yellow";
    }
    public void setPurple(){
        lightController.set(.89);
        color = "Purple";
    }
    public void setGreen(){
        lightController.set(.71);
        color = "Green";
    }
    public String getColor(){
        return color;
    }
}
