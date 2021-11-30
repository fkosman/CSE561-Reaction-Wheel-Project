package Component.CSE561project;

import java.util.Random;

import Component.SISO.siso;
import GenCol.doubleEnt;
import GenCol.entity;
import model.modeling.content;
import model.modeling.message;

public class Disturbance extends siso {

	private static final double MAX = 0.00000005;
	private static final double MIN = 0.00000001;
	private static final double FREQ = 0.05;
	private static final double DURATION = 2.0;
	private static final double TIME_STEP = 0.1;
	
	protected static Random rand = new Random();
	protected double torque;
	protected boolean passive;
	protected int timeRemaining;
	
	public Disturbance(){
	    super("Disturbance");
		addOutport("SatellitePort");
		
		initalize();
	}
	
	public void initalize(){
		passive = true;
		torque = 0;
		timeRemaining = 0;
		holdIn("active", 1);
		super.initialize();
	}
	
	public void  deltext(double e, message x){
		Continue(e);
	}
	
	public void deltint(){
		if (passive)	{
			double prob = rand.nextDouble();
			if (prob < FREQ)	{
				passive = false;
				int coinflip = rand.nextInt(2);
				torque = (MIN + rand.nextDouble()*(MAX - MIN));
				if (coinflip == 1)
					torque = -1 * torque;

				timeRemaining = (int)(DURATION / TIME_STEP);
			}
		}
		else	{
			timeRemaining--;
			if (timeRemaining == 0)	{
				passive = true;
				torque = 0.0;
			}
		}
		
		holdIn("active", 1);
	}
	
	public message out(){
		message m = new message();
		content con = makeContent("SatellitePort", new doubleEnt(torque));
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
