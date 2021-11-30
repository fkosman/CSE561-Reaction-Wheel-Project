package Component.CSE561project;

import GenCol.doubleEnt;
import GenCol.entity;
import model.modeling.message;
import view.modeling.ViewableAtomic;

public class Generator extends ViewableAtomic {
	private static final String START = "Start";
	//private static final String SEL = "Select";
	private static final String COMMAND = "Command Out";
	//private static final String DISTURB = "Impulse Disturbance";
	private static final String PASSIVE = "passive";
	private static final String WAIT = "Wait";
	private static final String ACTIVE = "active";
	
	double test_val = 15.0;
	
	
	public Generator() {this("Generator");}
	public Generator(String name)
	{
		super(name);
		
		addInport(START);
		//addInport(SEL);
		
		addOutport(COMMAND);
		
		addTestInput(START, new entity("Start"));
		addTestInput(START, new entity("Stop"));
	}
	
	public void initialize()
	{
		passivate();
	}
	
	public void deltext(double e, message x)
	{
		sigma = sigma - e;
		if(phaseIs(PASSIVE))
		{
			for(int i=0; i<x.getLength(); i++)
			{
				String in = x.getValOnPort(START, i).toString();
				System.out.println(in);
				if(in.equals("Start"))
				{
					holdIn(WAIT, 2.0);
				}
			}
		}
		else
		{
			for(int i=0; i<x.getLength(); i++)
			{
				String in = x.getValOnPort(START, i).toString();
				if(in.equals("Stop"))
				{
					passivate();
				}
				
			}
		}
	}
	
	public void deltint()
	{
		if(phaseIs(WAIT))
		{
			holdIn(ACTIVE,0);
		}
		else if(phaseIs(ACTIVE))
		{
			passivate();
		}
	}
	
	public message out()
	{
		message m = new message();
		if(phase == ACTIVE)
		{
			m.add(makeContent(COMMAND, new doubleEnt(test_val)));
			
		}
		return m;
	}
}
