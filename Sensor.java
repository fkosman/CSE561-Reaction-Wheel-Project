package Component.CSE561project;

import Component.SISO.siso;
import GenCol.doubleEnt;
import GenCol.*;
import model.modeling.content;
import model.modeling.message;

public class Sensor extends siso {
	public entity input;
	
	public double spacecraft_angle;
	public double spacecraft_rot_speed;
	public double wheel_tor;
	public double external_tor;

	private static final double time_step=0.1;
	private static final double init_angle=0;
	// Moment of Inertia for a cube passing through the center:
	// I = (1/6)ma^2, m = mass(kg), a=length of side(m), I(kgm^2)
	private static final double spacecraft_inertia = (1.0/6.0)*1.33*0.01*0.01;
	
	public Sensor(){
	    super("Sensor");
		addInport("MotorPort");
		addInport("ExternalPort");
		addOutport("ControllerPortAngle");
		addOutport("ControllerPortVelocity");
		addOutport("UserPort");
		
		addTestInput("ExternalPort", new doubleEnt((double)1000000));
		addTestInput("ExternalPort", new doubleEnt((double)-1000000));
		
		initalize();
	}
	
	public void initalize(){
		spacecraft_angle = init_angle;
		spacecraft_rot_speed = 0;
		wheel_tor = 0;
		external_tor = 0;
		holdIn("Active", 1);
		super.initialize();
	}
	
	public void  deltext(double e, message x){
		for(int i=0; i<x.getLength(); i++){
			if (messageOnPort(x,"ExternalPort",i)){
				input = x.getValOnPort("ExternalPort", i);
				external_tor = Double.parseDouble(input.toString());
			}
			else if (messageOnPort(x,"MotorPort",i)){
				input = x.getValOnPort("MotorPort", i);
				wheel_tor = Double.parseDouble(input.toString());
			}
		}
		Continue(e);
	}
	
	public void deltint(){
		double net_tor = external_tor - wheel_tor;
		spacecraft_rot_speed += (net_tor / spacecraft_inertia) * time_step;	
		spacecraft_angle += spacecraft_rot_speed * time_step;
		
		spacecraft_angle = spacecraft_angle % (2 * Math.PI);
		if (spacecraft_angle < 0.0)
			spacecraft_angle += 2 * Math.PI;
				
		holdIn("Active", 1);
	}
	
	public message out(){
		message m = new message();
		content angle_con = makeContent("ControllerPortAngle", new doubleEnt(spacecraft_angle));
		content vel_con = makeContent("ControllerPortVelocity", new doubleEnt(spacecraft_rot_speed));
		content con = makeContent("UserPort", new entity("Disturbance: " + 
														 String.format("%.2f", external_tor) + " Nm"));
		m.add(angle_con);
		m.add(vel_con);
		m.add(con);
		return m;
	}
	
	public void showState(){
		super.showState();
	}
	
	public String getTooltipText(){
		   return
		   super.getTooltipText();
		  }
}
