package slam;

import utils.Pose;
import lejos.util.Matrix;

public class AugmentedState extends Matrix {
	private static final long serialVersionUID = 1L;

	public AugmentedState(Matrix m) {
		super(m.getArray());
	}
	public AugmentedState(int N) throws IllegalArgumentException {
		super(3*N + 3, 1);
	}
	
	public Pose getRobotPose() {
		Pose mu = new Pose((float)this.get(0, 0), (float)this.get(1, 0),
				(float)this.get(2, 0));
		return mu;
	}
	
	public Pose getLandmark(int j) {
		int base = 3 + 3*j;
		Pose z = new Pose((float)this.get(base, 0), (float)this.get(base + 1, 0),
				(float)this.get(base + 2, 0));
		return z;
	}
	
	public void addToLandmarkPose(int j, float x, float y) {
		int base = 3 + 3*j;
		this.set(base, 0, this.get(base, 0) + x);
		this.set(base + 1, 0, this.get(base + 1, 0) + y);
	}
	
	public void setRobotPose(float x, float y) {
		this.set(0, 0, x);
		this.set(1, 0, y);
	}
	
	public void setLandmarkPose(int j, float x, float y) {
		int base = 3 + 3*j;
		this.set(base, 0, x);
		this.set(base + 1, 0, y);
	}
}
