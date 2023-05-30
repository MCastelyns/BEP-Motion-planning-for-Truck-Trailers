using MathWorks.MATLAB.NET.Arrays;
using MathWorks.MATLAB.NET.Utility;

class Program
{
    static void Main()
    {
        // Create an instance of our MATLAB compiled class MyMPC
        MyMPC.Class1 matlabFunc = new MyMPC.Class1();

        // Example inputs
        double[] inputs = new double[] {3.14, 42, 1.23, 2.34};

        // Loop for each time step
        for (int i = 0; i < 10; i++)
        {
            // Call the MATLAB function 'my_mpc'
            MWArray[] result = matlabFunc.my_mpc(1, inputs[0], inputs[1], inputs[2], inputs[3]);

            // Print the results
            Console.WriteLine("Output1 at time step {0}: {1}", i, result[0]);
            Console.WriteLine("Output2 at time step {0}: {1}", i, result[1]);

            // Update inputs for the next time step as needed
            // inputs = ...
        }
    }
}
