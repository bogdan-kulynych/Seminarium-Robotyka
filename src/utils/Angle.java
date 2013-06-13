package utils;

public class Angle {
	
	//inner representation - in radians
	private float phi;
	
	//angle in degrees
	public Angle(float phi){
		this(phi,true);
	}
	
	public Angle(float phi, boolean inRadians){
		if(inRadians){
			this.phi = phi;
		} else {
			this.phi = rad(phi);
		}
	}
	
	public void set(Angle angle){
		this.phi = angle.phi;
	}
	
	public Angle add(float rad){
		return new Angle(this.phi + rad);
	}
	
	public Angle add(Angle angle){
		return this.add(angle.phi);
	}
	
	public Angle sub(float rad){
		return new Angle(this.phi - rad);
	}
	
	public Angle sub(Angle angle){
		return this.sub(angle.phi);
	}
	
	public float rad(){
		return normalize(phi);
	}
	
	public float deg(){
		return deg(normalize(phi));
	}
	
	public boolean lt(Angle other){
		return normalize(phi)<normalize(other.phi);
	}
	
	public boolean gt(Angle other){
		return normalize(phi)>normalize(other.phi);
	}
	
	//====================================
	
	public static Angle LEFT = new Angle(1);
	public static Angle RIGHT = new Angle(-1);
	
	public static float PI = (float)Math.PI;
	public static float TWOPI = 2*PI;
	public static float PIdeg = 180;
	public static float TWOPIdef = 2*PIdeg;
	
	public static float rad (float deg){
		return deg/180*PI;
	}
	
	public static float deg (float rad){
		return rad/PI*180;
	}
	
	private static float normalize(float angle,float pi){
		float twopi = 2*pi;
		angle = angle %twopi;
		angle = (angle + twopi) % twopi;
		if(angle > pi){
			angle -= twopi;
		}
		return angle;
	}
	
	private static float normalize(float rad){
		return normalize(rad,PI);
	}	
}
