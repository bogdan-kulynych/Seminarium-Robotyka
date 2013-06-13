package utils;


import lejos.util.Matrix;

public class Eigen {
	private Matrix matrix;
	private double[][] eigenvectors;
	private double[] eigenvalues;
	
	/**
	 * Assumes matrix is 2x2
	 */
	Eigen(Matrix matrix) {
		this.matrix = matrix;
	}
	
	static double det(Matrix a) {
		return a.get(0, 0) * a.get(1, 1)
			 - a.get(0, 1) * a.get(1, 0);
	}
	
	void calculate() {
		eigenvalues = new double[2];
		double a = matrix.get(0, 0);
		double b = matrix.get(0, 1);
		double c = matrix.get(1, 0);
		double d = matrix.get(1, 1);
		double gap = Math.sqrt(a*a - 2*a*d + 4*b*c + d*d);
		eigenvalues[0] = (-gap + a + d) / 2;
		eigenvalues[1] = (gap + a + d) / 2;
		
		eigenvectors = new double[2][];
		eigenvectors[0] = new double[2];
		eigenvectors[1] = new double[2];
		eigenvectors[0][0] =  -(gap - a + d) / (2 * c);
		eigenvectors[1][0] =  -(-gap - a + d) / (2 * c);
		eigenvectors[0][1] =  1;
		eigenvectors[1][1] =  1;
	}
	
	double[][] getEigenVectors() {
		return eigenvectors;
	}
	
	double[] getEigenVector(int i) {
		return eigenvectors[i];
	}
	
	double[] getEigenValues() {
		return eigenvalues;
	}
	
	double getEigenValue(int i) {
		return eigenvalues[i];
	}
}

