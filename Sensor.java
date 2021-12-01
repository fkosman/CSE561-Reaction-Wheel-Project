package Component.CSE561project;

import Component.SISO.siso;
import GenCol.doubleEnt;
import GenCol.*;
import model.modeling.content;
import model.modeling.message;

public class Sensor extends siso {
	private double currentAngle;
	private double angularVelocity;
	private double wheelTorque;
	private double externalTorque;

	private static final double TIME_STEP = 0.1;
	private static final double INITIAL_ANGLE = 0.0;
	private static final double INITIAL_VELOCITY = 0.0;
	
	// Moment of Inertia for a cube passing through the center:
	// I = (1/6)ma^2, m = mass(kg), a=length of side(m), I(kgm^2)
	private static final double INERTIA = (1.0/6.0)*1.33*0.01*0.01;
	
	public Sensor(){
	    super("Sensor");
		addInport("MotorPort");
		addInport("ExternalPort");
		addOutport("ControllerPortAngle");
		addOutport("ControllerPortVelocity");
		addOutport("UserPort");
		
		initalize();
	}
	
	public void initalize(){
		super.initialize();
		wheelTorque = 0;
		externalTorque = 0;
		currentAngle = INITIAL_ANGLE;
		angularVelocity = INITIAL_VELOCITY;
		holdIn("active", 1);
	}
	
	public void  deltext(double e, message x){
		Continue(e);
		for(int i=0; i<x.getLength(); i++)	{
			if (messageOnPort(x,"ExternalPort",i))	{
				doubleEnt portVal = (doubleEnt) x.getValOnPort("ExternalPort", i);
				externalTorque = portVal.getv();
			}
			if (messageOnPort(x,"MotorPort",i))	{
				doubleEnt portVal = (doubleEnt) x.getValOnPort("MotorPort", i);
				wheelTorque = portVal.getv();
			}
		}
	}
	
	public void deltint(){
		double net_tor = externalTorque - wheelTorque;
		angularVelocity += (net_tor / INERTIA) * TIME_STEP;	
		currentAngle += angularVelocity * TIME_STEP;
		
		currentAngle = currentAngle % (2 * Math.PI);
		if (currentAngle < 0.0)
			currentAngle += 2 * Math.PI;
				
		holdIn("active", 1);
	}
	
	//Okay to use default confluent delta function
	public void deltcon(double e, message x)
	{
		deltext(0,x);
		deltint();
	}
	
	public message out(){
		showState();
		message m = new message();
		content angleCon = makeContent("ControllerPortAngle", new doubleEnt(currentAngle));
		content velocityCon = makeContent("ControllerPortVelocity", new doubleEnt(angularVelocity));
		content disturbanceCon = makeContent("UserPort", new doubleEnt(externalTorque));
		m.add(angleCon);
		m.add(velocityCon);
		m.add(disturbanceCon);
		return m;
	}
	
	public void showState(){
		super.showState();
		System.out.println("Disturbance: " + String.format("%.2f", externalTorque) + " Nm");
	}
	
	public String getTooltipText(){
		   return
		   super.getTooltipText();
		  }
}
