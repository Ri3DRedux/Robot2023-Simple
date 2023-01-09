package frc.robot;

import java.lang.reflect.Field;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class LoopTimeLogger implements Runnable {
    private NetworkTable table;
    Map<String, Long> robotEpochs;
    Map<String, Long> cmdEpochs;

    public LoopTimeLogger(Robot robot) {
        /**
         * By registering as a task, we get:
         * Init Calls
         * Periodic calls (both auto/teleop and Robot)
         * 
         * You don't get the time in updating SmartDashboard, LiveWindow, Shuffleboard, and Simulation
         */
        SmartDashboard.postListenerTask(this);
        table = NetworkTableInstance.getDefault().getTable("/task_timings_ms");
        try{
            //read the robot class for the watchdog
            Class<?> f = robot.getClass().getSuperclass().getSuperclass();
            Field field = f.getDeclaredField("m_watchdog");
            field.setAccessible(true);  //Make it accessible so you can access it
            Watchdog watchDog = (Watchdog)field.get(robot);    // At last it's yours.
            robotEpochs = watchToMap(watchDog);

            var cs = CommandScheduler.getInstance();
            f = cs.getClass();
            field = f.getDeclaredField("m_watchdog");
            field.setAccessible(true);
            watchDog = (Watchdog)field.get(cs);
            cmdEpochs = watchToMap(watchDog);
        }
        catch (Exception e){
            DriverStation.reportError("Task Timing could NOT be initialized.", false);
        }
    }

    @SuppressWarnings("unchecked")
    public Map<String, Long> watchToMap(Watchdog watchDog) throws IllegalArgumentException, IllegalAccessException, NoSuchFieldException, SecurityException{
        //from the watchdog, get the tracer
        var watchClass = watchDog.getClass();
        var field = watchClass.getDeclaredField("m_tracer");
        field.setAccessible(true);
        Tracer tracer = (Tracer)field.get(watchDog);

        //from the tracer. get the records
        var tracerClass = tracer.getClass();
        field = tracerClass.getDeclaredField("m_epochs");
        field.setAccessible(true);
        return (Map<String, Long>)field.get(tracer);
    }

    @Override
    public void run() {
        if(robotEpochs != null) {
            var totalTime = 0.f;
            for (var key : robotEpochs.keySet()) {
                var entry = table.getEntry(key);
                var time = robotEpochs.get(key)/1000f;
                entry.setFloat(time);
                totalTime += time;
            }
            table.getEntry("Loop_Time").setFloat(totalTime);
        }

        if(cmdEpochs != null) {
            for (var key : cmdEpochs.keySet()) {
                var entry = table.getEntry(key);
                var time = cmdEpochs.get(key)/1000f;
                entry.setFloat(time);
            }
        }

        //restart the task each loop it runs
        SmartDashboard.postListenerTask(this);
    }
}
