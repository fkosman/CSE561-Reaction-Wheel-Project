package Component.CSE561project;

import Component.SISO.siso;
import GenCol.*;
import model.modeling.*;

public class Motor extends siso {
	
	private double angularVelocity;
	private double actualTorque;
	private double commandedTorque;
	
	private static final double MAX_VELOCITY =  2 * Math.PI * 6500 / 60; // 6500rpm to rad/s
	private static final double MAX_ACCELERATION =  20.0;
	private static final double INERTIA = 0.001; 
	private static final double MAX_TORQUE = MAX_ACCELERATION * INERTIA; 
	private static final double TIME_STEP = 0.1;
	private static final double INITIAL_VELOCITY = 0.0;
	
	public Motor()	{
		super("Motor");
		addInport("ControllerPort");
		addOutport("SensorPort");
		addOutport("UserPort");
		
		initialize();
	}
	
	public void initialize()	{
		super.initialize();
		actualTorque = 0.0;
		commandedTorque = 0.0;
		angularVelocity = INITIAL_VELOCITY;
		holdIn("active",1);
	}
	
	public void deltext(double e, message x)
	{
		Continue(e);

		for(int i=0; i<x.getLength(); i++) {
			doubleEnt portVal = (doubleEnt) x.getValOnPort("ControllerPort", i);
			commandedTorque = portVal.getv();
		}
	}
	
	public void deltint()
	{
		double prevAngularVelocity = angularVelocity;
		
		if(Math.abs(commandedTorque) > MAX_TORQUE) {
			if(commandedTorque > 0)
				angularVelocity += (MAX_TORQUE / INERTIA) * TIME_STEP;
			else
				angularVelocity -= (MAX_TORQUE / INERTIA) * TIME_STEP;
		}
		else
			angularVelocity += (commandedTorque / INERTIA) * TIME_STEP;
		
		if(Math.abs(angularVelocity) > MAX_VELOCITY)	{
			if(angularVelocity > 0)
				angularVelocity = MAX_VELOCITY;
			else
				angularVelocity = -1.0 * MAX_VELOCITY;
		}
		
		actualTorque = (angularVelocity - prevAngularVelocity) * INERTIA / TIME_STEP;
		
		holdIn("active", 1);
	}
	
	//Okay to use default confluent delta function
	public void deltcon(double e, message x)	{
		deltext(0,x);
		deltint();
	}
	
	public message out()	{
		showState();
		message m = new message();
		m.add(makeContent("SensorPort", new doubleEnt(actualTorque)));
		m.add(makeContent("UserPort", new doubleEnt(60 * angularVelocity / (2 * Math.PI))));
		return m;
	}
	
	public void showState()	{
		super.showState();
		System.out.println("Wheel velocity: " + 
							String.format("%.2f", 60 * angularVelocity / (2 * Math.PI)) + "RPM");
	}
}
