package Component.CSE561project;

import Component.SISO.siso;
import GenCol.doubleEnt;
import GenCol.entity;
import model.modeling.*;

public class Motor extends siso {

	private static String ACTIVE_PHASE = "active";
	private static String CMD_TRQ_INPORT = "commanded torque";
	private static String RCTN_TRQ_OUTPORT = "reaction torque";
	
	private static double MAX_TRQ = 0.0032; //Nm
	private static double MAX_VEL = 6500/60*2*Math.PI; //6500rpm to rad/s
	private static double TIME_STEP = 1.0;
	//    L = I*omega, L = max torque storage, omega is wheel angular velocity
	// => I_max = L/omega_max
	private static double INERTIA = 0.02 / MAX_VEL;
	
	private double wheel_vel;
	private double cmd_trq;
	public double reaction_trq;
	
	public Motor(){this("Motor", 0);} //TODO default initial wheel spin
	public Motor(String name, double init_wheel_vel)
	{
		super(name);
		
		wheel_vel = init_wheel_vel;
		
		addInport(CMD_TRQ_INPORT);
		addOutport(RCTN_TRQ_OUTPORT);
		addOutport("UserPort");
		
		addTestInput(CMD_TRQ_INPORT, new doubleEnt((double)1000000));
		addTestInput(CMD_TRQ_INPORT, new doubleEnt((double)-1000000));
		
		initialize();
	}
	
	public void initialize()
	{
		holdIn(ACTIVE_PHASE,1);
	
		reaction_trq = 0.0;
		super.initialize();
	}
	
	public void deltext(double e, message x)
	{
		Continue(e);

		for(int i=0; i<x.getLength(); i++) 
		{
			doubleEnt portVal = (doubleEnt) x.getValOnPort(CMD_TRQ_INPORT, i);
			cmd_trq = portVal.getv();
		}
		if(Math.abs(cmd_trq) > MAX_TRQ)
		{
			if(cmd_trq > 0)
			{
				cmd_trq = MAX_TRQ;
			}
			else
			{
				cmd_trq = -1.0 * MAX_TRQ;
			}
		}
	}
	
	public void deltint()
	{
		double tmp_wheel_vel = wheel_vel;
		wheel_vel += cmd_trq/INERTIA;
		if(Math.abs(wheel_vel) > MAX_VEL)
		{
			if(wheel_vel > 0)
			{
				wheel_vel = MAX_VEL;
			}
			else
			{
				wheel_vel = -1.0 * MAX_VEL;
			}
		}
		reaction_trq = INERTIA * (wheel_vel - tmp_wheel_vel) * TIME_STEP;
		
		holdIn(ACTIVE_PHASE, 1);
	}
	
	public message out()
	{
		message m = new message();
		m.add(makeContent(RCTN_TRQ_OUTPORT, new doubleEnt(reaction_trq)));
		m.add(makeContent("UserPort", new doubleEnt(wheel_vel)));
		return m;
	}
	
	public void showState()
	{
		super.showState();
		System.out.println("Wheel Vel: " + Double.toString(wheel_vel));
		System.out.println("Wheel Torqe: " + Double.toString(reaction_trq));
	}
}
