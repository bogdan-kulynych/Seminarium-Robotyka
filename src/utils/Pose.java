package utils;

public class Pose {
	public Point position;
	public Angle direction;
	
	public Pose(float x, float y, float rad){
		position = new Point(x, y);
		direction = new Angle(rad);
	}
	public Pose(Point z, Angle rotation){
		this.position = z;
		this.direction = rotation;
	}
	
	public void set(Pose pose){
		this.position.set(pose.position);
		this.direction.set(pose.direction);
	}
	
	public String toString() {
		return position.x + ", " + position.y + ", " + direction.deg();
	}
}
