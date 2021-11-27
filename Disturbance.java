package Component.CSE561project;

import java.util.Random;

import Component.SISO.siso;
import GenCol.doubleEnt;
import GenCol.entity;
import model.modeling.content;
import model.modeling.message;

public class Disturbance extends siso {

	private static final int magnitude=3;
	private static final double freq=0.0003;
	private static final double duration=2;
	private static final double time_step=0.001;
	
	protected static Random rand = new Random();
	protected double torque;
	protected boolean passive;
	protected int timeRemaining;
	
	public Disturbance(){
	    super("Disturbance");
		addOutport("SensorPort");
		
		initalize();
	}
	
	public void initalize(){
		passive = true;
		torque = 0;
		timeRemaining = 0;
		holdIn("Active", 1);
		super.initialize();
	}
	
	public void  deltext(double e, message x){
		Continue(e);
	}
	
	public void deltint(){
		if (passive)	{
			double prob = rand.nextDouble();
			if (prob < freq)	{
				passive = false;
				int coinflip = rand.nextInt(2);
				if (coinflip == 1)
					torque = -1 * (rand.nextInt(magnitude) + 1);
				else
					torque = rand.nextInt(magnitude) + 1;
				timeRemaining = (int)(duration / time_step);
			}
		}
		else	{
			timeRemaining--;
			if (timeRemaining == 0)	{
				passive = true;
				torque = 0.0;
			}
		}
		
		holdIn("Active", 1);
	}
	
	public message out(){
		message m = new message();
		content con = makeContent("SensorPort", new doubleEnt(torque));
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
