using System;
using System.Diagnostics;

class Program
{
    static void Main()
    {
        int input = 1;

        for (int i = 0; i < 10; i++)
        {
            // Set up the Python process start info
            // This will capture anything that is 'printed' to the console in the MPC.py process and redirect it to our C# file, note that this data is send as a string
            // So we have to first treat it like a string, and then convert it to an int using int.Parse()
            ProcessStartInfo psi = new ProcessStartInfo
            {
                FileName = "python",
                Arguments = $"C:/Users/Mitchel/Documents/GitHub/self-driving-truck-trailer/ConsoleAppPython/MPC.py {input}",
                RedirectStandardOutput = true, 
                UseShellExecute = false,
                CreateNoWindow = true
            };

            // Start the Python process
            using (Process pythonProcess = new Process())
            {
                pythonProcess.StartInfo = psi;
                pythonProcess.Start();

                // Read the output from the Python process, right now it reads everything, but should look into implementing ways to read per int or float
                // .ReadToEnd() -> leest hele output van 1 keer python runnen
                // .ReadLine() -> leest 1 print statement, dus per lijn, denk dat dit handigst is, dan kunnen we ints/floats los importen
                string output = pythonProcess.StandardOutput.ReadLine().Trim();
                string output2 =pythonProcess.StandardOutput.ReadLine().Trim();

                // Print what is happening, for debugging for now
                Console.WriteLine($"Input to Python from C#: {input}");
                Console.WriteLine($"Output from Python to C#: {output}");
                Console.WriteLine($"Output from Python to C#: {output2}");

                // Determine the new input to be sent to MPC.py for the next iteration. Just a test to see if it could perform arithmetics
                input = int.Parse(output) + 1;

                // Cleanup resources (not required when using 'using' statement), because it automatically closes when we are no longer using it. If we implement it differently, we might have to use it again
                // pythonProcess.Dispose();
            }
        }
    }
}


