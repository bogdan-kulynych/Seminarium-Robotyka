package remoterobot;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.awt.image.BufferStrategy;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import javax.swing.JFrame;

import lejos.util.Matrix;
import slam.EKFSLAM;
import utils.Eigen;
import utils.Point;
import utils.Pose;

public class MapGUI extends JFrame {

    public float scale = 200.0f;
    public Point2D translate = new Point2D.Double(0.0f, 0.0f);
    public java.util.List<Point3D> states = new LinkedList<Point3D>();
    public java.util.List<Point2D> path = new LinkedList<Point2D>();
    public LinkedList<LinkedList<Point2D>> measurements = new LinkedList<LinkedList<Point2D>>();
    public Map<Integer,EKFPos> landmarks = new HashMap<Integer,EKFPos>();
    
    public static class EKFPos {
    	public double[] v1;
    	public double[] v2;
    	public Point mu;
    	
    	public EKFPos(){
    		v1 = new double[2];
    		v2 = new double[2];
    		mu = new Point(0,0);
    	}
    	
    	public EKFPos(Point mu, Matrix sigma){
    		this();
    		update(mu,sigma);
    	}
    	
    	public void update(Point mu, Matrix sigma){
    		this.mu = mu;
    		System.out.println("Sigma for landmark:");
    		System.out.println(EKFSLAM.showMatrix(sigma));
    		Eigen e = new Eigen(sigma);
    		e.calculate();
    		v1 = e.getEigenVector(0);
    		v1[0]=v1[0]/Math.sqrt(v1[0]*v1[0]+v1[1]*v1[1])*Math.sqrt(e.getEigenValue(0));
    		v1[1]=v1[1]/Math.sqrt(v1[0]*v1[0]+v1[1]*v1[1])*Math.sqrt(e.getEigenValue(0));
    		
    		v2 = e.getEigenVector(1);
    		v2[0]=v2[0]/Math.sqrt(v2[0]*v2[0]+v2[1]*v2[1])*Math.sqrt(e.getEigenValue(1));
    		v2[1]=v2[1]/Math.sqrt(v2[0]*v2[0]+v2[1]*v2[1])*Math.sqrt(e.getEigenValue(1));
    	}
    }
    
    class Robot {
        Point2D pos = new Point2D.Double(0.0f, 0.0f);
        double rot = 0; 
    } 
    
    public EKFPos robotEKF = new EKFPos();
    
    public Robot robot = new Robot();
    
    public MapGUI() {
        path.add(new Point2D.Double(robot.pos.getX(), robot.pos.getY()));
        states.add(new Point3D(robot.pos.getX(), robot.pos.getY(), robot.rot));
        measurements.add(new LinkedList<Point2D>());
        
    }

    private static final long DEMO_TIME = 5000;
    
    public void run() {
        setBackground(Color.black);
        setFont(new Font("Dialog", 0, 24));
        this.setBounds(10, 10, 800, 600);
        this.setVisible(true);
    }
    
    private void setCamera() {
        int[] pos = cast(robot.pos.getX(), robot.pos.getY());
        double[] max = {robot.pos.getX(), robot.pos.getY()};
        double[] min = {robot.pos.getX(), robot.pos.getY()};
        
        if(pos[0] < 30 || pos[1] < 30 || 
                pos[1] > getBounds().height - 30 ||
                pos[0] > getBounds().width - 30) {
            double x = robot.pos.getX(), y = robot.pos.getY();
            for(Point3D p : states) {
                x += p.getX();
                y += p.getY();
                double[] temp = {p.getX(), p.getY()};
                if(temp[0] < min[0])
                    min[0] = temp[0];
                if(temp[1] < min[1])
                    min[1] = temp[1];
                if(temp[0] > max[0])
                    max[0] = temp[0];
                if(temp[1] > max[1])
                    max[1] = temp[1];
            }
            x /= (states.size()+1);
            y /= (states.size()+1);
            
            translate = new Point2D.Double(x, y);
            
            int[] minP = cast(min[0], min[1]);
            int[] maxP = cast(max[0], max[1]);
            
            if(minP[0] < 0) {
                scale *= (double)getBounds().width/(getBounds().width - 2.2 * minP[0]);
                minP = cast(min[0], min[1]);
                maxP = cast(max[0], max[1]);
            }
            if(minP[1] < 0) {
                scale *= (double)getBounds().height/(getBounds().height - 2.2 * minP[1] + 100);
                minP = cast(min[0], min[1]);
                maxP = cast(max[0], max[1]);
            }
            if(maxP[1] > getBounds().height) {
                scale *= (double)getBounds().height/(- getBounds().height + 2.2 * maxP[1] + 100);
                minP = cast(min[0], min[1]);
                maxP = cast(max[0], max[1]);
            }
            if(maxP[0] > getBounds().width) {
                scale *= (double)getBounds().width/(- getBounds().width + 2.2 * maxP[0] + 100);
            }
        }
    }
    
    public void moveRobot(double r) {
        robot.pos.setLocation(r * Math.cos(robot.rot) + robot.pos.getX(), 
                r * Math.sin(robot.rot) + robot.pos.getY());
        path.add(new Point2D.Double(robot.pos.getX(), robot.pos.getY()));
        draw();
    }
    
    public void warpRobot(Pose pose){
    	robot.pos.setLocation(pose.position.x, pose.position.y);
    	robot.rot = pose.direction.rad();
    	draw();    	
    }
    
    public void rotateRobot(double fi) {
        robot.rot = robot.rot + fi;
        draw();
    }
    
    public void addState(LinkedList<Point2D> z, LinkedList<Integer> signatures) {
        states.add(new Point3D(robot.pos.getX(), robot.pos.getY(), robot.rot));
        measurements.add(z);
        draw();
    }
    
    //-------------------------------
    // Draw methods
    //-------------------------------
    
    private int cast(double x) {
        return (int)(x*scale);
    }
    
    private int[] cast(double x, double y) {
        return new int[]{
            (int)(  getBounds().width/2 + ( - translate.getX() + x)*scale),
            (int)(  getBounds().height/2 + ( - translate.getY()+ y)*scale),
        };
    }
    
    private double[] cast(int x, int y) {
        return new double[]{
            (double)( translate.getX() + ( x - getBounds().width/2)/scale),
            (double)( translate.getY() + ( y - getBounds().height/2)/scale),
        };
    }


    private void drawGrid(Graphics g) {
        g.setColor(Color.white);
        double[] ul = cast(0, 0);
        double[] dr = cast(getBounds().width, getBounds().height);
        
        for(double i = Math.ceil(ul[0]); i<dr[0]; i++) {
            int x = cast(i, 0.0f)[0];
            g.drawLine(x, 0, x, getBounds().height);
        }
        for(double i = Math.ceil(ul[1]); i<dr[1]; i++) {
            int x = cast(0.0f, i)[1];
            g.drawLine(0, x, getBounds().width, x);
        }
    }
    
    private void drawPath(Graphics g) {
        g.setColor(Color.gray);
        int size = path.size();
        for(int i=1; i<size; i++) {
            Point2D p1 = path.get(i-1);
            int[] pos1 = cast(p1.getX(), p1.getY());
            Point2D p2 = path.get(i);
            int[] pos2 = cast(p2.getX(), p2.getY());
            g.drawLine(pos1[0], pos1[1], pos2[0], pos2[1]);
        }
    }
    
    
    private void drawStates(Graphics g) {
        int size = states.size();
        
        for(int i=0; i<size; i++) {
            g.setColor(Color.gray);
            Point3D p2 = states.get(i);
            int[] pos2 = cast(p2.getX(), p2.getY());
            //System.out.println("p2 "+p2.getX()+" "+p2.getY());
            g.drawOval(pos2[0]-15, pos2[1]-15, 30, 30);
            g.drawLine(pos2[0], pos2[1], (int)(15 * Math.cos(p2.Z) + pos2[0]), 
                    (int)(15 * Math.sin(p2.Z) + pos2[1]));
            if(i!=0) {
                Point3D p1 = states.get(i-1);
                int[] pos1 = cast(p1.getX(), p1.getY());
                g.drawLine(pos1[0], pos1[1], pos2[0], pos2[1]);
            }
            g.setColor(Color.red);
            LinkedList<Point2D> z = measurements.get(i);
            for(Point2D zi : z) {
                g.drawLine(pos2[0], pos2[1], (int)(cast(zi.getX()) * Math.cos(zi.getY()+p2.Z) + pos2[0]), 
                        (int)(cast(zi.getX()) * Math.sin(zi.getY()+p2.Z) + pos2[1]));
            }
        }
    }
    
    private void drawRobot(Graphics g) {
        g.setColor(Color.yellow);
        int[] pos = cast(robot.pos.getX(), robot.pos.getY());
        int[] x1 = cast(0.1 * Math.cos(robot.rot) + robot.pos.getX(), 
                0.1 * Math.sin(robot.rot) + robot.pos.getY());
        int[] x2 = cast(0.05 * Math.cos(robot.rot+Math.PI*2/3) + robot.pos.getX(), 
                0.05 * Math.sin(robot.rot+Math.PI*2/3) + robot.pos.getY());
        int[] x3 = cast(0.05 * Math.cos(robot.rot+Math.PI*4/3) + robot.pos.getX(), 
                0.05 * Math.sin(robot.rot+Math.PI*4/3) + robot.pos.getY());
        
        
        g.drawLine(x1[0], x1[1], x2[0], x2[1]);
        g.drawLine(x1[0], x1[1], x3[0], x3[1]);
        g.drawLine(pos[0], pos[1], x2[0], x2[1]);
        g.drawLine(pos[0], pos[1], x3[0], x3[1]);
    }
    
    private void drawEKFEllipse(Point mu, double[] v1, double[] v2, Graphics2D g){
    	
    	double angle = Math.atan2(v1[1], v1[0]);
    	
    	double length1 = cast(Math.sqrt(v1[0]*v1[0]+v1[1]*v1[1])),
    			length2 = cast(Math.sqrt(v2[0]*v2[0]+v2[1]*v2[1]));
    	
    	int[] coord = cast(mu.x,mu.y);
    	Ellipse2D ellipse = new Ellipse2D.Double(coord[0]-length1/2.0,coord[1]-length2/2.0,length1,length2);
    	
    	AffineTransform at = new AffineTransform();
    	at.rotate(angle,coord[0],coord[1]);
    	
    	g.draw(at.createTransformedShape(ellipse));
    }
    
    private void drawRobotEKF(Graphics2D g){
    	g.setColor(Color.cyan);
    	
    	drawEKFEllipse(robotEKF.mu, robotEKF.v1, robotEKF.v2, g);
    }
    
    private void drawLandmarksEKF(Graphics2D g){
    	g.setColor(Color.orange);
    	for(EKFPos e : landmarks.values()){
    		int[] coord = cast(e.mu.x,e.mu.y);
    		Ellipse2D c = new Ellipse2D.Double(coord[0]-2.5,coord[1]-2.5,5,5);
    		g.draw(c);
    		System.out.println("EigenVecs: "+e.v1[0]+" "+e.v1[1]+", "+e.v2[0]+" "+e.v2[1]);
    		drawEKFEllipse(new Point(e.mu.x, e.mu.x), e.v1, e.v2, g);
    	}
    }
    
    public void draw() {
        setCamera();
        BufferStrategy strategy = getBufferStrategy();
        Graphics g = strategy.getDrawGraphics();
        paint(g);
        g.dispose();
        strategy.show();
    }
    
    public void paint(Graphics g) {
        g.fillRect(0, 0, getBounds().width, getBounds().height);
        drawGrid(g);
        drawPath(g);
        drawStates(g);
        drawRobot(g);
        drawRobotEKF((Graphics2D)g);
        drawLandmarksEKF((Graphics2D)g);
    }
}

