package Component.CSE561project;

import view.modeling.ViewableAtomic;
import model.modeling.message;
import GenCol.doubleEnt;


public class Controller extends ViewableAtomic
{
	//secondary phase:
	double goal_angle;
	double actual_angle;
	double GAIN;
	doubleEnt commanded_torque;
	double TOL;
	
	public Controller() {this("Controller");}
	public Controller(String name) 
	{
		super(name);
		
		//Two in ports and one out port
		addInport("UserPort");
		addInport("FeedbackPort");
		addOutport("MotorCmdPort");
		
		//test inputs:
		for(int i=1; i<=8 ;i++)
		{
			double angle = ((double)i/4.0)*Math.PI;
			addTestInput("UserPort", new doubleEnt(angle));
			addTestInput("FeedbackPort", new doubleEnt(angle));
		}
		
		initialize();
	}
	
	
	public void initialize()
	{
		goal_angle = 0;
		actual_angle = 0;
		GAIN = 1;
		TOL = 0.035;
		commanded_torque = new doubleEnt(0.0);
		holdIn("active", 1);
	}
	
	public void deltext(double e, message x) {
		sigma = sigma - e;
		for(int i=0; i<x.getLength(); i++)
		{
			if(messageOnPort(x, "UserPort", i))
			{
				goal_angle = Double.parseDouble(x.getValOnPort("UserPort", i).toString());
			} 
			else if(messageOnPort(x,"FeedbackPort",i))
			{
				actual_angle = Double.parseDouble(x.getValOnPort("FeedbackPort", i).toString());
			}
		}
	}
	
	//
	public void deltint() {
		sigma = 1;
		if(Math.abs(goal_angle - actual_angle) > TOL)
			commanded_torque = new doubleEnt(-1 * GAIN * (goal_angle - actual_angle));
		else
			commanded_torque = new doubleEnt(0.0);
	}
	
	//Okay to use default confluent delta function
	public void deltcon(double e, message x)
	{
		deltint();
		deltext(0,x);
	}
	
	//Output the commanded Torque to the motor.
	public message out()
	{
		message m = new message();
		m.add(makeContent("MotorCmdPort", commanded_torque));
		return m;
	}
}
