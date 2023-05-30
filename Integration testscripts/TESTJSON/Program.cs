using Newtonsoft.Json; //https://documentation.help/Json.NET/N_Newtonsoft_Json.htm
using System;
using System.Diagnostics;
using System.IO;

class Program
{
    static void Main()
    {
        Stopwatch stopWatch = Stopwatch.StartNew(); // Start timing

        // Initialize a 2D array (200x5) as test data to be sent to the Python script, chose this amount because 200 nodes of 5 states is probably a good reference value
        float[][] inputData = new float[200][];
        for (int i = 0; i < 200; i++)
        {
            inputData[i] = new float[] { (i + 1) * 0.1f, (i + 1) * 0.2f, (i + 1) * 0.3f, (i + 1) * 0.4f, (i + 1) * 0.5f };
        }

        // Serialize the inputData into a JSON string and write it to the input file. This is efficient especially for large amounts of data
        string inputFilePath = "input.json";
        File.WriteAllText(inputFilePath, JsonConvert.SerializeObject(inputData));

        // Set up the Python process start info. This tells the operating system how to start the new process
        ProcessStartInfo psi = new ProcessStartInfo
        {
            FileName = "python",
            Arguments = $"JSONTEST.py {inputFilePath}",
            RedirectStandardOutput = true, // Allows us to read the output of the Python process, which will be the name of the file with the data (the results from python)
            UseShellExecute = false, // Required to redirect input/output/error streams, it will send us possible errors in the python file now instead of trying to display it in the python execution window
            CreateNoWindow = true // Starts the process without creating a new window, we don't need a window
        };

        

        // Start the Python process and handle the returned data
        using (Process process = new Process())
        {
            process.StartInfo = psi;
            process.Start();

            // Read the output file name from the Python process
            string outputFilePath = process.StandardOutput.ReadLine().Trim();

            // Deserialize the JSON string from the output file back into a 2D array of floats
            float[][] outputData = JsonConvert.DeserializeObject<float[][]>(File.ReadAllText(outputFilePath));

            // Print the first and last array to console for testing
            Console.WriteLine($"Output from Python to C#: {string.Join(", ", outputData[0])}");
            Console.WriteLine($"Output from Python to C#: {string.Join(", ", outputData[199])}");
        }
       stopWatch.Stop(); // Stop timing

        // Get the elapsed time as a TimeSpan value.
        TimeSpan ts = stopWatch.Elapsed;

        // Format and display the TimeSpan value.
        string elapsedTime = $"{ts.Hours:00}:{ts.Minutes:00}:{ts.Seconds:00}.{ts.Milliseconds / 10:00}";
        Console.WriteLine("RunTime " + elapsedTime);
    }
}
