package Component.CSE561project;

import view.modeling.ViewableAtomic;
import model.modeling.message;
import GenCol.*;


public class Controller extends ViewableAtomic
{
	//secondary phase:
	double goal_angle;
	double actual_angle;
	double rot_speed;
	double GAIN;
	double commanded_torque;
	double TOL;
	private static double TIME_STEP = 0.1;
	private static double ROT_TIME = 5.0;
	
	public Controller() {this("Controller");}
	public Controller(String name) 
	{
		super(name);
		
		//Two in ports and one out port
		addInport("UserPort");
		addInport("AngleFeedbackPort");
		addInport("VelocityFeedbackPort");
		addOutport("MotorCmdPort");
		addOutport("UserFeedback");
		
		//test inputs:
		addTestInput("UserPort", new doubleEnt(0.0));
		addTestInput("UserPort", new doubleEnt(10.0));
		addTestInput("UserPort", new doubleEnt(-10.0));
		addTestInput("UserPort", new doubleEnt(180.0));
		
		initialize();
	}
	
	
	public void initialize()
	{
		goal_angle = 0;
		actual_angle = 0;
		rot_speed = 0;
		GAIN = 0.0001;
		TOL = 0.00005;
		commanded_torque = 0.0;
		holdIn("active", 1);
	}
	
	public void deltext(double e, message x) {
		sigma = sigma - e;
		for(int i=0; i<x.getLength(); i++)
		{
			if(messageOnPort(x, "UserPort", i))
			{
				goal_angle = Double.parseDouble(x.getValOnPort("UserPort", i).toString()) % 360.0;
				if (goal_angle < 0.0)
					goal_angle = 360.0 + goal_angle;
				
				goal_angle *= (2 * Math.PI / 360.0);
			} 
			else if(messageOnPort(x,"AngleFeedbackPort",i))	
				actual_angle = Double.parseDouble(x.getValOnPort("AngleFeedbackPort", i).toString());
			else if (messageOnPort(x,"VelocityFeedbackPort",i))
				rot_speed = Double.parseDouble(x.getValOnPort("VelocityFeedbackPort", i).toString());
		}
	}
	
	public void deltint() {
		sigma = 1;
		
		double clockWiseDist, antiClockWiseDist;
		if (goal_angle > actual_angle)	{
			clockWiseDist = Math.abs(goal_angle - actual_angle);
			antiClockWiseDist = Math.abs((goal_angle - 2 * Math.PI) - actual_angle);
		}
		else	{
			clockWiseDist = Math.abs(goal_angle + (2 * Math.PI - actual_angle));
			antiClockWiseDist = Math.abs(actual_angle - goal_angle);
		}
		
		double angle_gap = Math.min(clockWiseDist, antiClockWiseDist);
		double ideal_rot_speed = angle_gap / ROT_TIME;
		
		if (antiClockWiseDist < clockWiseDist)
			ideal_rot_speed *= -1.0;
		
		if(Math.abs(ideal_rot_speed - rot_speed) > TOL 
		   || Math.abs(goal_angle - actual_angle) > TOL)	
			commanded_torque = -1 * (ideal_rot_speed - rot_speed) * GAIN;
		else
			commanded_torque = 0.0;
	}
	
	//Okay to use default confluent delta function
	public void deltcon(double e, message x)
	{
		deltext(0,x);
		deltint();
	}
	
	//Output the commanded Torque to the motor.
	public message out()
	{
		message m = new message();
		m.add(makeContent("MotorCmdPort", new doubleEnt(commanded_torque)));
		m.add(makeContent("UserFeedback", new entity("Angle: " +
													 String.format("%.2f", actual_angle * 360 / (2 * Math.PI)) +
													  " degrees")));
		return m;
	}
}
