package frc.robot.utilities;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.util.ArrayDeque;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.Filesystem;

public class FileWriteUtil {

    private static ArrayDeque<String> messageQueue;
    private static ArrayDeque<String> printQueue;
    private static final Object lock = new Object();

    private FileWriteUtil () {}

    static {
        messageQueue = new ArrayDeque<>(200);
        printQueue = new ArrayDeque<>(200);

        new Thread(FileWriteUtil::handle).start();
        // log("File Util started!");
    }

    private static void handle () {
        File file = new File(Filesystem.getOperatingDirectory().getAbsolutePath() + "/data.csv");

        // try {
        //     if (Files.deleteIfExists(file.toPath())) {
        //         System.out.println("Deleted: " + file.getAbsolutePath());
        //     } else {
        //         System.out.println("Couldn't delete: " + file.getAbsolutePath());
        //     }
        //     // Files.createFile(file.toPath());
        // } catch (IOException e1) {
        //     e1.printStackTrace();
        // }

        try (PrintWriter writer = new PrintWriter(file)) {
            while (true) {
                synchronized (lock) {
                    ArrayDeque<String> tmp = printQueue;
                    printQueue = messageQueue;
                    messageQueue = tmp;
                }
    
                if (printQueue.size() > 0) {
                    writer.println(printQueue.stream().collect(Collectors.joining("\r\n")));
                    writer.flush();
                    printQueue.clear();
                }
                
                try { Thread.sleep(500); }
                catch (Exception e) { }
            }
        } catch (FileNotFoundException e1) {
            e1.printStackTrace();
        }

    }

    public static void log (String message) {
        synchronized (lock) {
            messageQueue.add(message);
        }
    }
    
}