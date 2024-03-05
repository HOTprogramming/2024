package org.hotutilites.hotlogger;

import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class HotLogger
{
    public static final String LOGS_DIRECTORY = "/home/lvuser/logs/";

    public static final String DELIMITER = ",";
    public static final String EMPTY = "";
    public static final Double ROW_TIMEOUT_SECONDS = .02;
    public static final Double LOG_PERIOD_SECONDS = 1.0;
    public static final Integer ALLOWED_LOG_FILES = 5;

    private static boolean onNewRow = true;
    private static Map<String, String> currentRow = new LinkedHashMap<String, String>();
    private static double currentRowTimeFPGA;
    private static double currentRowTimeMatch;
    private static double currentRowTime;

    private static Notifier logScheduler = new Notifier(LogThread::WriteToFile);

    /*This code generates a log from the robot code and takes in the inputs specified by HotLogger.Setup
    * Creates new file every time code is uploaded, otherwise appends until code is re-uploaded*/
    public static void Setup(Object... valsToLog)
    {
        /*Clear current file string*/
        currentRow.clear();
        /*Create new header for the file*/
        StringBuilder headerBuilder = new StringBuilder();
        headerBuilder.append("FPGA Time").append(DELIMITER).append("Match Time").append(DELIMITER);
        for (int i = 0; i < valsToLog.length; ++i)
        {
            if (valsToLog[i] instanceof String)
            {
                currentRow.put((String) valsToLog[i], EMPTY);
                headerBuilder.append((String) valsToLog[i]).append(DELIMITER);
            }
            else if (valsToLog[i] instanceof List)
            {
                try
                {
                    for (String s : (List<String>) valsToLog[i])
                    {
                        currentRow.put(s, EMPTY);
                        headerBuilder.append(s).append(DELIMITER);
                    }
                }
                catch (Exception ignored)
                {
                }
            }
        }

        logScheduler.stop();
        LogThread.SetHeader(headerBuilder.toString());
        LogQueue.RestartQueue();
        logScheduler.startPeriodic(LOG_PERIOD_SECONDS);
    }
/*turn any value input into a double*/
    public static void Log(String key, Integer value)
    {
        Log(key, Double.valueOf(value));
    }
/* turn any input value into a string*/
    public static void Log(String key, Double value)
    {
        String stringValue = " ";
        try
        {
            stringValue = String.valueOf(value);
        }
        catch (Exception ignored)
        {
        }
        Log(key, stringValue);
    }

    public static void Log(String key, String value)
    {
        if (!currentRow.containsKey(key))
            return;

        if (onNewRow)
        {
            currentRowTimeFPGA = Timer.getFPGATimestamp();
            currentRowTimeMatch = Timer.getMatchTime();
            /*get match time only works during a practice or real match, use FPGA timer if match time not available*/
            if (currentRowTimeMatch == -1)
            {
                currentRowTime = currentRowTimeFPGA;
            }
            else
            {
                currentRowTime = currentRowTimeMatch;
            }
        }

        if (!currentRow.get(key).equals(EMPTY))
        {
            PushCurrentRow();
            Log(key, value);
            return;
        }
        else
            currentRow.put(key, value);

        if (Timer.getFPGATimestamp() - currentRowTimeFPGA > ROW_TIMEOUT_SECONDS)
        {
            PushCurrentRow();
        }
    }

    private static void PushCurrentRow()
    {
        try
        {
            LinkedHashMap<String, String> newMap = new LinkedHashMap<>();
            for (Map.Entry<String, String> currentEntry : currentRow.entrySet())
            {
                newMap.put(currentEntry.getKey(), currentEntry.getValue());
            }
            /*newMap.put("Match Time",String.valueOf(currentRowTimeMatch));*/
            LogRow row = new LogRow(newMap, String.valueOf(currentRowTimeFPGA+DELIMITER+currentRowTimeMatch));
            
            LogQueue.PushToQueue(row);
        }
        catch (Exception ignored)
        {
        }
        for (Map.Entry<String, String> entry : currentRow.entrySet())
        {
            entry.setValue(EMPTY);
        }
        onNewRow = true;
    }

    private static class LogRow
    {
        public final Map<String, String> Values;
        public final String TimeStamp;

        public LogRow(Map<String, String> values, String logTimeStamp)
        {
            this.Values = values;
            this.TimeStamp = logTimeStamp;
        }
    }

    private static class LogThread
    {
        private static FileWriter fileWriter = null;
        private static String headerToOutput = null;

        public static synchronized void SetHeader(String header)
        {
            headerToOutput = header;
        }

        /**
         * Function called only by logging thread
         * Creates new file every time code is uploaded
         */
        public static synchronized void WriteToFile()
        {
            try
            {
                List<LogRow> logQueue = LogQueue.FlushQueue();

                if (logQueue == null || logQueue.size() == 0)
                    return;

                StringBuilder output;
                
                if (headerToOutput != null)
                {
                    output = new StringBuilder(headerToOutput);
                    output.append("\n");
                    headerToOutput = null;
                }
                else
                    output = new StringBuilder();
    
                for (int i = 0; i < logQueue.size(); ++i)
                {
                    LogRow row = logQueue.get(i);
                    output.append(row.TimeStamp).append(DELIMITER);
                    for (Map.Entry<String, String> e : row.Values.entrySet())
                    {
                        output.append(e.getValue()).append(DELIMITER);
                    }
                    if (i + 1 < logQueue.size())
                        output.append("\n");
                }


                if (output.toString().trim().isEmpty())
                    return;
                
                /*Create new file name based on current date, time*/
                    String fileName = LOGS_DIRECTORY
                        + new SimpleDateFormat("yyyyMMdd_HH_mm_ss").format(LogQueue.GetDate()) + ".txt";

                File f = new File(fileName);
                /*If file name doesn't exist, create a new file*/
                if (!f.exists())
                {
                    File directory = new File(LOGS_DIRECTORY);

                    /*Create a matrix of all the files in the log directory*/
                    directory.mkdirs();
                    File[] files = directory.listFiles();

                    /*If there are more files in folder than designated amount, sort them and delete the oldest */
                    if (files.length > ALLOWED_LOG_FILES)
                    {
                        Arrays.sort(files, new Comparator<File>()
                        {

                            @Override
                            /*This sorts the files by last modified, oldest one is at top of list*/
                            public int compare(File arg0, File arg1)
                            {
                                if (arg0.lastModified() < arg1.lastModified())
                                    return 1;
                                else if (arg0.lastModified() > arg1.lastModified())
                                    return -1;
                                else
                                    return 0;
                            }
                        });
                        /*Delete the one at the top of the list; sorted above so the oldest one modified is at the top*/
                        for (int i = ALLOWED_LOG_FILES - 1; i < files.length; ++i)
                        {
                            files[i].delete();
                        }
                    }

                    f.createNewFile();
                    if (fileWriter != null)
                        fileWriter.close();

                    fileWriter = new FileWriter(f);
                }

                if (fileWriter == null)
                    fileWriter = new FileWriter(f);
                fileWriter.append(output+"\n");
                /*fileWriter.append("\n");*/
                fileWriter.flush();
            }
            catch (Exception e)
            {
                e.printStackTrace();
            }
        }

        public static synchronized void CloseStream()
        {
            if (fileWriter != null)
                try
                {
                    fileWriter.close();
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }
        }
    }

    public static void CloseStream()
    {
        LogThread.CloseStream();
        logScheduler.stop();
    }

    private static class LogQueue
    {
        /**
         * Log structures shared between threads
         */
        private static List<LogRow> logQueue = new ArrayList<>();
        private static Date logDate = new Date();

        private static synchronized List<LogRow> FlushQueue()
        {
            List<LogRow> tmpQueue = new ArrayList<>();
            tmpQueue.addAll(logQueue);
            logQueue.clear();
            return tmpQueue;
        }

        private static synchronized Date GetDate()
        {
            return logDate;
        }

        public static synchronized void PushToQueue(LogRow row)
        {
            logQueue.add(row);
        }

        public static synchronized void RestartQueue()
        {
            logDate = new Date();
            logQueue.clear();
        }
    }

	public static void Log(String key, boolean b)
	{
        Log(key, (b) ? "true" : "false");
	}
}
