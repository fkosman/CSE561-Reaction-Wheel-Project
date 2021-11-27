package Component.CSE561project;

import java.awt.*;

import GenCol.doubleEnt;
import view.modeling.*;

public class Satellite extends ViewableDigraph {

	public static String CMD_IN_PORT = "Command In";
	public static String DISTURBANCE_IN_PORT = "Disturbance In";
	public static String ANGLE_OUT_PORT = "Angle Out";
	
	public Satellite() {this("Satellite");}
	public Satellite(String name)
	{
		super(name);
		
		addInport(CMD_IN_PORT);
		addInport(DISTURBANCE_IN_PORT);
		
		addOutport(ANGLE_OUT_PORT);
		addOutport("Wheel Speed Out");
		addOutport("Disturbance Out");
		
		Motor motor = new Motor();
		Sensor sensor = new Sensor();
		Controller controller = new Controller();
		Disturbance disturbance = new Disturbance();
		
		add(motor);
		add(sensor);
		add(controller);
		
		addCoupling(this, CMD_IN_PORT, controller, "UserPort");
		addCoupling(this, DISTURBANCE_IN_PORT, sensor, "ExternalPort");
		
		addCoupling(controller, "MotorCmdPort", motor, "commanded torque");
		
		addCoupling(motor, "reaction torque", sensor, "MotorPort");
		addCoupling(motor, "UserPort", this, "Wheel Speed Out");
		
		addCoupling(sensor, "ControllerPort", this, ANGLE_OUT_PORT);
		addCoupling(sensor, "ControllerPort", controller, "FeedbackPort");
		addCoupling(sensor, "UserPort", this, "Disturbance Out");
		
		for(int i=1; i<=8 ;i++)
		{
			double angle = ((double)i/4.0)*Math.PI;
			addTestInput(CMD_IN_PORT, new doubleEnt(angle));
		}
		
		
		
		initialize();
	}
	
	
    /**
     * Automatically generated by the SimView program.
     * Do not edit this manually, as such changes will get overwritten.
     */
    @Override
    public void layoutForSimView()
    {
        preferredSize = new Dimension(499, 276);
        ((ViewableComponent)withName("Motor")).setPreferredLocation(new Point(-1, 110));
        ((ViewableComponent)withName("Controller")).setPreferredLocation(new Point(11, 23));
        ((ViewableComponent)withName("Sensor")).setPreferredLocation(new Point(-4, 208));
    }
}
