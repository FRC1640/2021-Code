package frc.robot.utilities;

import java.util.ArrayDeque;
import java.util.stream.Collectors;

public class Logger {

    private static ArrayDeque<String> messageQueue;
    private static ArrayDeque<String> printQueue;
    private static final Object lock = new Object();

    private Logger () {}

    static {
        messageQueue = new ArrayDeque<>(200);
        printQueue = new ArrayDeque<>(200);

        new Thread(Logger::handle).start();
        log("Logger started!");
    }

    private static void handle () {
        while (true) {
            synchronized (lock) {
                ArrayDeque<String> tmp = printQueue;
                printQueue = messageQueue;
                messageQueue = tmp;
            }

            if (printQueue.size() > 0) {
                System.out.println(printQueue.stream().collect(Collectors.joining("\r\n")));
                printQueue.clear();
            }
            
            try { Thread.sleep(500); }
            catch (Exception e) { }
        }
    }

    public static void log (String message) {
        synchronized (lock) {
            messageQueue.add(message);
        }
    }
    
}