package Component.CSE561project;

import GenCol.doubleEnt;
import model.modeling.message;
import view.modeling.ViewableAtomic;

public class Transducer2 extends ViewableAtomic {
	private static final String GEN_INPT = "Gen Input";
	private static final String SEN_INPT = "Sensor Input";
	private static final String RISE_TIME = "Rise Time";
	
	public double goal_angle;
	public double init_angle;
	public double rise_angle;
	public boolean collect_init;
	public boolean start;
	public double current_angle;
	public double clock;
	public double start_time, end_time;
	public int rot_dir;
	
	public Transducer2()	{
		super("Transducer");
		
		addInport(GEN_INPT);
		addInport(SEN_INPT);
		addOutport(RISE_TIME);
	}
	
	public void initialize()	{
		holdIn("Active", 1.0);
		clock = 0;
		goal_angle = 0;
		rise_angle = 0;
		init_angle = 0;
		collect_init = false;
		current_angle = 0;
		start = false;
		rot_dir = 0;
		super.initialize();
	}
	
	public void deltext(double e, message x)	{
		clock = clock + e;
		Continue(e);
		for(int i=0; i<x.getLength(); i++)	{
			if(messageOnPort(x, GEN_INPT, i))	{
				goal_angle = ((doubleEnt)x.getValOnPort(GEN_INPT, i)).getv();
				if(goal_angle < 0.0)
					goal_angle = 360.0 + goal_angle;
				collect_init = true;
				rot_dir = 0;
				start_time = clock;
			}
			if(messageOnPort(x, SEN_INPT, i))
				current_angle = ((doubleEnt)x.getValOnPort(SEN_INPT, i)).getv();
		}
		if(collect_init)	{
			init_angle = current_angle;
			collect_init = false;
			
			double clockWiseDist, antiClockWiseDist;
			if (goal_angle > init_angle)		{
				clockWiseDist = Math.abs(goal_angle - init_angle);
				antiClockWiseDist = Math.abs((goal_angle - 360.0) - init_angle);
			}
			else	{
				clockWiseDist = Math.abs(goal_angle + (360.0 - init_angle));
				antiClockWiseDist = Math.abs(init_angle - goal_angle);
			}
			
			double angle_gap = Math.min(clockWiseDist, antiClockWiseDist);
			if(clockWiseDist > antiClockWiseDist)	{
				rot_dir = 1;
				rise_angle = goal_angle + (0.1 * angle_gap);
				if(rise_angle > 360.0)	{
					rise_angle -= 360.0;
				}
			}
			else	{
				rot_dir = -1;
				rise_angle = goal_angle - (0.1 * angle_gap);
				if(rise_angle < 0.0)
					rise_angle += 360.0;
			}
			holdIn("Testing", 1.0);
		}
	}
	
	public void deltint()	{
		clock += sigma;
		if(phaseIs("Testing"))	{
			if(rot_dir < 0 && current_angle >= rise_angle)
				initialize();
			else if(rot_dir > 0 && current_angle <= rise_angle)
				initialize();
		}

	}
	
	public message out()	{
		message m = new message();
		
		if(phaseIs("Testing"))	{
			if(rot_dir < 0 && current_angle >= rise_angle)
				m.add(makeContent(RISE_TIME, new doubleEnt(clock - start_time)));
			else if(rot_dir > 0 && current_angle <= rise_angle)
				m.add(makeContent(RISE_TIME, new doubleEnt(clock - start_time)));
		}
		
		return m;
	}
}
