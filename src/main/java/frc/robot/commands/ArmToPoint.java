package frc.robot.commands;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.ArmPathPlanner;
import frc.robot.util.Point;

public class ArmToPoint extends CommandBase {
    
    private ElbowSubsystem m_elbow;
    private ShoulderSubsystem m_shoulder;

    private Point target;
    private Point next;

    private ArmPathPlanner planner = new ArmPathPlanner(16.5, 20.5);

    public ArmToPoint(ElbowSubsystem elbow, ShoulderSubsystem shoulder, Point target) {
        this.m_elbow = elbow;
        this.m_shoulder = shoulder;

        this.target = target;
        double currentX = 16.5 * Math.cos(Math.toRadians(m_shoulder.getCurrentAngle())) + 
            20.5 * Math.cos(Math.toRadians(m_elbow.getCurrentAngle()));
        double currentY = 16.5 * Math.sin(Math.toRadians(m_shoulder.getCurrentAngle())) + 
            20.5 * Math.sin(Math.toRadians(m_elbow.getCurrentAngle()));
        
        double closestX = currentX - currentX % 5.0 + 5.0;
        double closestY = currentY - currentY % 5.0 + 5.0;
        
        this.next = new Point(closestX, closestY); 

        addRequirements(elbow);
        addRequirements(shoulder);
    }

    @Override
    public void execute() {
        Vector<N2> armAngles = planner.getArmAngles(next);
        if (armAngles.get(0, 0) >= 0 && armAngles.get(0, 0) <= 90) {
            m_shoulder.setTarget(armAngles.get(0,0));
        }
        if (armAngles.get(1, 0) >= 0 && armAngles.get(1, 0) <= 180) {
            m_elbow.setTarget(armAngles.get(1,0));
        }
    }

    @Override
    public boolean isFinished() {
        return (m_elbow.atTarget() && m_shoulder.atTarget());
    }

}
