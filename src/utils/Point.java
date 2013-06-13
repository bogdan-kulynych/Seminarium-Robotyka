package utils;

public class Point {
	public float x, y;
	
	public Point(float x, float y){
		this.x = x;
		this.y = y;
	}
	
	public Point add(Point point){
		return new Point(this.x + point.x, this.y + point.y);
	}
	
	public void set(Point point){
		this.x = point.x;
		this.y = point.y;
	}
}
