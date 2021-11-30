package Component.CSE561project;

import view.modeling.ViewableAtomic;
import model.modeling.message;
import GenCol.*;


public class Controller extends ViewableAtomic	{
	private double goalAngle;
	private double actualAngle;
	private double angularVelocity;
	private double commandedTorque;
	
	private static final double GAIN = 0.0001;
	private static final double TOL = 0.005;
	private static final double TIME_STEP = 0.1;
	
	// Minimum speed is to prevent the controller from having its
	// ideal speed converge to zero before it is close enough to the target.
	// A value of TOL * TIME_STEP is chosen to make sure it detects
	// when it has approached the tolerance margin of the target.
	private static final double MIN_SPEED = TOL * TIME_STEP; 
	private static final double ROT_TIME = 5.0;
	private static final double INITIAL_ANGLE = 0.0;
	private static final double INITIAL_VELOCITY = 0.0;
	
	public Controller() {
		super("Controller");
		addInport("UserPort");
		addInport("SensorPortAngle");
		addInport("SensorPortVelocity");
		addOutport("MotorPort");
		addOutport("UserFeedback");
		
		//test inputs:
		addTestInput("UserPort", new doubleEnt(0.0));
		addTestInput("UserPort", new doubleEnt(10.0));
		addTestInput("UserPort", new doubleEnt(-10.0));
		addTestInput("UserPort", new doubleEnt(180.0));
		
		initialize();
	}
	
	
	public void initialize()	{
		super.initialize();
		goalAngle = INITIAL_ANGLE;
		actualAngle = INITIAL_ANGLE;
		angularVelocity = INITIAL_VELOCITY;
		commandedTorque = 0.0;
		holdIn("active", 1);
	}
	
	public void deltext(double e, message x) {
		sigma = sigma - e;
		for(int i=0; i<x.getLength(); i++)	{
			if(messageOnPort(x, "UserPort", i))	{
				goalAngle = Double.parseDouble(x.getValOnPort("UserPort", i).toString()) % 360.0;
				if (goalAngle < 0.0)
					goalAngle = 360.0 + goalAngle;
				
				goalAngle *= (2 * Math.PI / 360.0);
			} 
			else if(messageOnPort(x,"SensorPortAngle",i))	
				actualAngle = Double.parseDouble(x.getValOnPort("SensorPortAngle", i).toString());
			else if (messageOnPort(x,"SensorPortVelocity",i))
				angularVelocity = Double.parseDouble(x.getValOnPort("SensorPortVelocity", i).toString());
		}
	}
	
	public void deltint() {
		sigma = 1;
		
		double posAngularDist, negAngularDist;
		if (goalAngle > actualAngle)	{
			posAngularDist = Math.abs(goalAngle - actualAngle);
			negAngularDist = Math.abs((goalAngle - 2 * Math.PI) - actualAngle);
		}
		else	{
			posAngularDist = Math.abs(goalAngle + (2 * Math.PI - actualAngle));
			negAngularDist = Math.abs(actualAngle - goalAngle);
		}
		
		double angularDist = Math.min(posAngularDist, negAngularDist);
		
		double idealAngularVelocity;
		if (angularDist < TOL)
			idealAngularVelocity = 0.0;
		else	
			idealAngularVelocity = Math.max(angularDist / ROT_TIME, MIN_SPEED);	
		
		
		if (negAngularDist < posAngularDist)
			idealAngularVelocity *= -1.0;
		
		commandedTorque = -1.0 * (idealAngularVelocity - angularVelocity) * GAIN;
	}
	
	//Okay to use default confluent delta function
	public void deltcon(double e, message x)	{
		deltext(0,x);
		deltint();
	}
	
	//Output the commanded Torque to the motor.
	public message out()	{
		showState();
		message m = new message();
		m.add(makeContent("MotorPort", new doubleEnt(commandedTorque)));
		m.add(makeContent("UserFeedback", new doubleEnt(actualAngle * 360 / (2 * Math.PI))));
		return m;
	}
	
	public void showState()	{
		super.showState();
		System.out.println("Angle: " + 
						   String.format("%.2f", actualAngle * 360 / (2 * Math.PI)) + " degrees");
	}
}
