package Component.CSE561project;

import GenCol.*;
import model.modeling.message;
import view.modeling.ViewableAtomic;

public class Transducer1 extends ViewableAtomic {
	private static final double TOL = 0.005 * 360 / (2 * Math.PI); // ~ 0.28 degrees
	private static final double WHEEL_INERTIA = 0.001;
	private static final double TIME_STEP = 0.1;
	
	public double goalAngle;
	public double actualAngle;
	public double clock;
	public double angleOffTime;
	public double disturbanceMomentum;
	public double wheelRotVelocity;
	
	double disturbanceVolume;
	double angleHitPercentage;
	
	public Transducer1(int observationTime)	{
		super("Transducer");
		addInport("DisturbancePort");
		addInport("SatellitePortAngle");
		addInport("SatellitePortWheel");
		
		addOutport("ClockResult");
		addOutport("DisturbanceResult");
		addOutport("WheelResult");
		addOutport("AngleHitResult");
		
		initialize(observationTime);
	}
	
	public void initialize(int observ)	{
		holdIn("active", observ);
		clock = 0.0;
		goalAngle = 0.0;
		actualAngle = 0.0;
		angleOffTime = 0.0;
		disturbanceMomentum = 0.0;
		super.initialize();
	}
	
	public void deltext(double e, message x)	{
		clock = clock + e;
		Continue(e);
		double tor = 0.0;
		for(int i=0; i<x.getLength(); i++)	{
			if(messageOnPort(x, "DisturbancePort", i))
				tor = ((doubleEnt)x.getValOnPort("DisturbancePort", i)).getv();
				if (tor != 0.0)	
					disturbanceMomentum += tor * TIME_STEP;
			if(messageOnPort(x, "SatellitePortAngle", i))	{
				actualAngle = ((doubleEnt)x.getValOnPort("SatellitePortAngle", i)).getv();
				double posAngularDist, negAngularDist;
				if (goalAngle > actualAngle)	{
					posAngularDist = Math.abs(goalAngle - actualAngle);
					negAngularDist = Math.abs((goalAngle - 360) - actualAngle);
				}
				else	{
					posAngularDist = Math.abs(goalAngle + (360 - actualAngle));
					negAngularDist = Math.abs(actualAngle - goalAngle);
				}
				
				double angularDist = Math.min(posAngularDist, negAngularDist);
				
				if (angularDist > TOL)
					angleOffTime += 1;
			}
			if(messageOnPort(x, "SatellitePortWheel", i))
				wheelRotVelocity = ((doubleEnt)x.getValOnPort("SatellitePortWheel", i)).getv();
		}
	}
	
	public void deltint()	{
		clock = clock + sigma;
	}
	
	public message out()	{
		clock += 1;
		
		wheelRotVelocity *= 2 * Math.PI / 60.0;
		double wheelMomentum = wheelRotVelocity * WHEEL_INERTIA;
		
		angleHitPercentage = 100.0 * (clock - angleOffTime) / clock;
		
		message m = new message();
		
		m.add(makeContent("ClockResult", new entity("Clock = " + clock)));
		
		m.add(makeContent("DisturbanceResult", new entity(String.format("Angular momentum created by disturbance = %6.3e", 
																		 disturbanceMomentum) + "kg*m^2/s")));
		
		m.add(makeContent("WheelResult", new entity(String.format("Angular momentum of reaction wheel = %6.3e", 
				   wheelMomentum) + "kg*m^2/s")));
		
		m.add(makeContent("AngleHitResult", new entity(String.format("Percent time angle was on target = %.4f", 
				   angleHitPercentage) + "%")));
		
		
		return m;
	}
}