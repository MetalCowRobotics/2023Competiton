package frc.robot.util;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class CalculateDistance{

    AnalogPotentiometer pot;
    MedianFilter medFilter = new MedianFilter(100);

    public CalculateDistance(int portNum){
        pot = new AnalogPotentiometer(portNum);
    }

    public double getValue(){
        return(medFilter.calculate(pot.get()));
    }

}