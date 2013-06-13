package remoterobot;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Point2D;
import java.io.IOException;
import static java.lang.Thread.sleep;
import java.util.LinkedList;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.*;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.remote.NXTCommand;
import lejos.pc.comm.NXTComm;
import lejos.pc.comm.NXTCommLogListener;
import lejos.pc.comm.NXTCommandConnector;
import lejos.pc.comm.NXTConnector;
import lejos.nxt.UltrasonicSensor;

/**
 * RemoteRobot contorls lego robots via Bluetooth. Does not need any specific
 * program on NXT.
 *
 * A - prawe kolo
 * B - lewe kolo
 * 
 * TODO: wymiary
 * --odleglosc pomiedzy kolami (srodkami)
 * --srednica kol
 * @author Grzegorz Jablonski
 *
 */
public class RemoteRobot extends JFrame {
    

    //TODO: dodac odometrie:
    //--proste podejscie - przyjmuje ze robot jedzie przod/tyl, skreca w miejscu
    //--trudniejsze podejscie - robot moze jezdzic po luku
    
    //Watek wykonywany na PC ktory co 200ms czyta dane:
    // --ilosc obrotow z silnikow A i B (kola)
    // --odleglosc pod odpowiednim katem
    // --zmienia kat glowicy
    
    // srednica robota 12.2 cm
    // srednica kola 4.4cm
    
    static class GUIThread extends Thread {
        
        int threadA = Motor.A.getTachoCount();
        int threadB = Motor.B.getTachoCount();
        UltrasonicSensor sonicThread;
        //public 

        public GUIThread(UltrasonicSensor st) {
            sonicThread = st;
        }

        @Override
        public void run() {
            while (true) {
                int dA = Motor.A.getTachoCount() - threadA;
                int dB = Motor.B.getTachoCount() - threadB;
                
                //System.out.println("distance = " + sonic.getDistance());
                if(FORWARD) {
                    if(STOP)
                        FORWARD = false;
                    
                    mapTest.moveRobot(Math.toRadians((dA+dB)/2)*0.036);
                    
                } 
                if(BACKWARD) {
                    if(STOP)
                        BACKWARD = false;
                    
                    mapTest.moveRobot(Math.toRadians((dA+dB)/2)*0.036);
                    
                } 
                if(LEFT) {
                    if(STOP)
                        LEFT = false;
                    
                    mapTest.rotateRobot( Math.toRadians((dB-dA)/2)*0.036 / 0.08);
                    
                } 
                if(RIGHT){
                    if(STOP)
                        RIGHT = false;
                    
                    mapTest.rotateRobot( Math.toRadians((dB-dA)/2)*0.036 / 0.08);
                    
                }
                if(STOP){
                    STOP = false;
                    SendControl();
                }
                
                threadA = Motor.A.getTachoCount();
                threadB = Motor.B.getTachoCount();
                
                
                try {
                    sleep(50);
                } catch (InterruptedException e) {
                }
            }

        }
    }
    // static variables
    public static int FORWARD_SPEED = 700;
    public static int ANGLE_SPEED = 200;
    public static boolean FORWARD, BACKWARD, LEFT, RIGHT;
    public static boolean IS_LEFT, IS_RIGHT, IS_FORWARD, IS_BACKWARD;
    public static boolean STOP = false;
    public static NXTConnector conn;
    public static UltrasonicSensor sonic;
    public static RemoteRobot.GUIThread guiThread;
    public static MapGUI mapTest;
    //end of static variables

    
    //called when key was pressed or released
    //wazne: pressed w systemach win jest wywolywane cykliczne gdy klawisz jest wcisniety
    //       released w systemach win jest wywolywane gdy puscimy klawisz
    //       released w ubuSendControl();ntu (i jemu podobnych) jest wywolywane cyklicznie, ale mozna to zmienic
    //       w ustawieniach systemu
    //       Dziala pod ubuntu ze zmienionymi ustawieniami
    public static void SendControl() {
        if (FORWARD || BACKWARD) {
            if (LEFT && !IS_LEFT) {
                Motor.B.setSpeed(ANGLE_SPEED);
                Motor.A.setSpeed(FORWARD_SPEED);
                IS_LEFT = true;
                IS_RIGHT = false;
                IS_FORWARD = IS_BACKWARD = false; // hack:/
            } else if (RIGHT && !IS_RIGHT) {
                Motor.B.setSpeed(FORWARD_SPEED);
                Motor.A.setSpeed(ANGLE_SPEED);
                IS_RIGHT = true;
                IS_LEFT = false;
                IS_FORWARD = IS_BACKWARD = false;
            } else if (!LEFT && !RIGHT) {
                Motor.A.setSpeed(FORWARD_SPEED);
                Motor.B.setSpeed(FORWARD_SPEED);
                IS_LEFT = false;
                IS_RIGHT = false;
                IS_FORWARD = IS_BACKWARD = false;
            }
        } else {
            if (LEFT) {
                Motor.A.setSpeed(ANGLE_SPEED);
                Motor.B.setSpeed(ANGLE_SPEED);
                Motor.A.forward();
                Motor.B.backward();
            } else if (RIGHT) {
                Motor.A.setSpeed(ANGLE_SPEED);
                Motor.B.setSpeed(ANGLE_SPEED);
                Motor.A.backward();
                Motor.B.forward();
            } else {
                Motor.A.setSpeed(FORWARD_SPEED);
                Motor.B.setSpeed(FORWARD_SPEED);
                Motor.A.stop();
                Motor.B.stop();
            }
        }

        if (FORWARD && !IS_FORWARD) {
            Motor.A.forward();
            Motor.B.forward();
            IS_FORWARD = true;
            IS_BACKWARD = false;

        } else if (BACKWARD && !IS_BACKWARD) {
            Motor.A.backward();
            Motor.B.backward();
            IS_FORWARD = false;
            IS_BACKWARD = true;
        } else if (!FORWARD && !BACKWARD && !LEFT && !RIGHT) {
            Motor.A.stop();
            Motor.B.stop();
            IS_FORWARD = IS_BACKWARD = false;
        }
    }//end SendControl
    //GUI
    public static JButton quit, connect;
    public static ButtonHandler bh = new ButtonHandler();

    public RemoteRobot() {
        setTitle("Control");
        setBounds(650, 350, 200, 100);
        setLayout(new GridLayout(2, 1));

        connect = new JButton(" Connect ");
        connect.addActionListener(bh);
        connect.addKeyListener(bh);
        add(connect);

        quit = new JButton("Quit");
        quit.addActionListener(bh);
        add(quit);

    }

    //bluetooth connection to NXT called "idefix"
    public static void connect() {
        conn = new NXTConnector();
        conn.addLogListener(new NXTCommLogListener() {
            public void logEvent(String message) {
                System.out.println(message);
            }

            public void logEvent(Throwable throwable) {
                System.err.println(throwable.getMessage());
            }
        });
        conn.setDebug(true);
        if (!conn.connectTo("btspp://idefix", NXTComm.LCP)) {
            System.err.println("Failed to connect");
            System.exit(1);
        }
        NXTCommandConnector.setNXTCommand(new NXTCommand(conn.getNXTComm()));

        initialize();
        //
    }

    public static void disconnect() {
        try {
            conn.close();
            System.exit(0);
        } catch (IOException ex) {
            Logger.getLogger(RemoteRobot.class.getName()).log(Level.SEVERE, null, ex);
        }

    }

    private static final int CLOSURE = 60;
    
    //set speeds, initialize sensors and starts thread that reads data from sensors
    public static void initialize() {
        Motor.A.setSpeed(FORWARD_SPEED);
        Motor.B.setSpeed(FORWARD_SPEED);

        Motor.C.setSpeed(900);
        Motor.C.smoothAcceleration(true);
        
        sonic = new UltrasonicSensor(SensorPort.S4);

        guiThread = new RemoteRobot.GUIThread(sonic);
        guiThread.start();
    }

    private static class ButtonHandler implements ActionListener, KeyListener {
        //***********************************************************************
        //Buttons action

        public void actionPerformed(ActionEvent ae) {
            if (ae.getSource() == quit) {
                disconnect();
            }
            if (ae.getSource() == connect) {
                connect();
            }

        }//End ActionEvent(for buttons)

        //***********************************************************************
        //Keyboard action
        public void keyPressed(KeyEvent ke) {
            if (ke.getKeyChar() == 'w') {
                FORWARD = true;
            }
            if (ke.getKeyChar() == 's') {
                BACKWARD = true;
            }
            if (ke.getKeyChar() == 'a') {
                LEFT = true;
            }
            if (ke.getKeyChar() == 'd') {
                RIGHT = true;
            }
            if (ke.getKeyChar() == 'f') {
                LinkedList<Point2D> z = new LinkedList<Point2D>();
                int right_angle_half = 20;
                int angle = 0;
                int steps = 10;
                for(int i = 0; i < steps; i++) {
                    int dist = sonic.getDistance();
                    if(dist < CLOSURE) {
                        z.add(new Point2D.Double((double)dist/100, Math.toRadians(angle-90)));
                    }
                    int old_angle = angle;
                    angle += right_angle_half;
                    angle %= steps * right_angle_half;
                    
                    Motor.C.rotate(angle - old_angle);
                    
                    try {
                        sleep(150);
                    } catch (InterruptedException ex) {
                        Logger.getLogger(RemoteRobot.class.getName()).log(Level.SEVERE, null, ex);
                    }
                }

                mapTest.addState(z);
            }
            SendControl();
        }//End keyPressed

        public void keyTyped(KeyEvent ke) {
        }//End keyTyped

        public void keyReleased(KeyEvent ke) {
            STOP = true;
        }//End keyReleased
    }//End ButtonHandler

    public static void main(String[] args) throws Exception {
        RemoteRobot NXTrc = new RemoteRobot();
        NXTrc.setVisible(true);
        NXTrc.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        
        mapTest = new MapGUI();
        mapTest.run();
        mapTest.createBufferStrategy(2);
        
    }
}