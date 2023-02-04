package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;

public class WristCommand extends CommandBase{
    CANSparkMax motor = new CANSparkMax(WristConstants.WRIST_MOTOR, MotorType.kBrushless);
    public void positive () {
        motor.set(1);
     }    
     public void negative () {
        motor.set(-1);
     }    
    }