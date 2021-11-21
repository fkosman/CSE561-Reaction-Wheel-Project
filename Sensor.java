package Component.CSE561project;

import Component.SISO.siso;
import GenCol.doubleEnt;
import GenCol.entity;
import model.modeling.content;
import model.modeling.message;

public class Sensor extends siso {
	public entity input;
	
	public double spacecraft_angle;
	public double spacecraft_rot_speed;
	public double wheel_tor;
	public double external_tor;

	private static final double time_step=0.000001;
	private static final double init_angle=0;
	private static final double spacecraft_inertia=10;
	
	public Sensor(){
	    super("Sensor");
		addInport("MotorPort");
		addInport("ExternalPort");
		addOutport("ControllerPort");
		
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
		spacecraft_rot_speed += (net_tor / spacecraft_inertia);	
		spacecraft_angle += (spacecraft_rot_speed * time_step);
				
		holdIn("Active", 1);
	}
	
	public message out(){
		message m = new message();
		content con = makeContent("ControllerPort", new doubleEnt(spacecraft_angle));
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
