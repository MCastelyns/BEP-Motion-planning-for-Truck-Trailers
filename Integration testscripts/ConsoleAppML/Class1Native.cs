/*
* MATLAB Compiler: 8.6 (R2023a)
* Date: Tue May 23 22:27:19 2023
* Arguments: "-B""macro_default""-W""dotnet:MyMPC,Class1""-T""link:lib""my_mpc.m"
*/
using System;
using System.Reflection;
using System.IO;
using MathWorks.MATLAB.NET.Arrays;
using MathWorks.MATLAB.NET.Utility;

#if SHARED
[assembly: System.Reflection.AssemblyKeyFile(@"")]
#endif

namespace MyMPCNative
{

  /// <summary>
  /// The Class1 class provides a CLS compliant, Object (native) interface to the MATLAB
  /// functions contained in the files:
  /// <newpara></newpara>
  /// C:\Users\Mitchel\Documents\GitHub\self-driving-truck-trailer\MyconsoleApp\my_mpc.m
  /// </summary>
  /// <remarks>
  /// @Version 1.0
  /// </remarks>
  public class Class1 : IDisposable
  {
    #region Constructors

    /// <summary internal= "true">
    /// The static constructor instantiates and initializes the MATLAB Runtime instance.
    /// </summary>
    static Class1()
    {
      if (MWMCR.MCRAppInitialized)
      {
        try
        {
          Assembly assembly= Assembly.GetExecutingAssembly();

          string ctfFilePath= assembly.Location;

		  int lastDelimiter = ctfFilePath.LastIndexOf(@"/");

	      if (lastDelimiter == -1)
		  {
		    lastDelimiter = ctfFilePath.LastIndexOf(@"\");
		  }

          ctfFilePath= ctfFilePath.Remove(lastDelimiter, (ctfFilePath.Length - lastDelimiter));

          string ctfFileName = "MyMPC.ctf";

          Stream embeddedCtfStream = null;

          String[] resourceStrings = assembly.GetManifestResourceNames();

          foreach (String name in resourceStrings)
          {
            if (name.Contains(ctfFileName))
            {
              embeddedCtfStream = assembly.GetManifestResourceStream(name);
              break;
            }
          }
          mcr= new MWMCR("",
                         ctfFilePath, embeddedCtfStream, true);
        }
        catch(Exception ex)
        {
          ex_ = new Exception("MWArray assembly failed to be initialized", ex);
        }
      }
      else
      {
        ex_ = new ApplicationException("MWArray assembly could not be initialized");
      }
    }


    /// <summary>
    /// Constructs a new instance of the Class1 class.
    /// </summary>
    public Class1()
    {
      if(ex_ != null)
      {
        throw ex_;
      }
    }


    #endregion Constructors

    #region Finalize

    /// <summary internal= "true">
    /// Class destructor called by the CLR garbage collector.
    /// </summary>
    ~Class1()
    {
      Dispose(false);
    }


    /// <summary>
    /// Frees the native resources associated with this object
    /// </summary>
    public void Dispose()
    {
      Dispose(true);

      GC.SuppressFinalize(this);
    }


    /// <summary internal= "true">
    /// Internal dispose function
    /// </summary>
    protected virtual void Dispose(bool disposing)
    {
      if (!disposed)
      {
        disposed= true;

        if (disposing)
        {
          // Free managed resources;
        }

        // Free native resources
      }
    }


    #endregion Finalize

    #region Methods

    /// <summary>
    /// Provides a single output, 0-input Objectinterface to the my_mpc MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// Define the variables
    /// </remarks>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object my_mpc()
    {
      return mcr.EvaluateFunction("my_mpc", new Object[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input Objectinterface to the my_mpc MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// Define the variables
    /// </remarks>
    /// <param name="input1">Input argument #1</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object my_mpc(Object input1)
    {
      return mcr.EvaluateFunction("my_mpc", input1);
    }


    /// <summary>
    /// Provides a single output, 2-input Objectinterface to the my_mpc MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// Define the variables
    /// </remarks>
    /// <param name="input1">Input argument #1</param>
    /// <param name="input2">Input argument #2</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object my_mpc(Object input1, Object input2)
    {
      return mcr.EvaluateFunction("my_mpc", input1, input2);
    }


    /// <summary>
    /// Provides a single output, 3-input Objectinterface to the my_mpc MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// Define the variables
    /// </remarks>
    /// <param name="input1">Input argument #1</param>
    /// <param name="input2">Input argument #2</param>
    /// <param name="input3">Input argument #3</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object my_mpc(Object input1, Object input2, Object input3)
    {
      return mcr.EvaluateFunction("my_mpc", input1, input2, input3);
    }


    /// <summary>
    /// Provides a single output, 4-input Objectinterface to the my_mpc MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// Define the variables
    /// </remarks>
    /// <param name="input1">Input argument #1</param>
    /// <param name="input2">Input argument #2</param>
    /// <param name="input3">Input argument #3</param>
    /// <param name="input4">Input argument #4</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object my_mpc(Object input1, Object input2, Object input3, Object input4)
    {
      return mcr.EvaluateFunction("my_mpc", input1, input2, input3, input4);
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the my_mpc MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// Define the variables
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] my_mpc(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "my_mpc", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 1-input Object interface to the my_mpc MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// Define the variables
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="input1">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] my_mpc(int numArgsOut, Object input1)
    {
      return mcr.EvaluateFunction(numArgsOut, "my_mpc", input1);
    }


    /// <summary>
    /// Provides the standard 2-input Object interface to the my_mpc MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// Define the variables
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="input1">Input argument #1</param>
    /// <param name="input2">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] my_mpc(int numArgsOut, Object input1, Object input2)
    {
      return mcr.EvaluateFunction(numArgsOut, "my_mpc", input1, input2);
    }


    /// <summary>
    /// Provides the standard 3-input Object interface to the my_mpc MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// Define the variables
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="input1">Input argument #1</param>
    /// <param name="input2">Input argument #2</param>
    /// <param name="input3">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] my_mpc(int numArgsOut, Object input1, Object input2, Object input3)
    {
      return mcr.EvaluateFunction(numArgsOut, "my_mpc", input1, input2, input3);
    }


    /// <summary>
    /// Provides the standard 4-input Object interface to the my_mpc MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// Define the variables
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="input1">Input argument #1</param>
    /// <param name="input2">Input argument #2</param>
    /// <param name="input3">Input argument #3</param>
    /// <param name="input4">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] my_mpc(int numArgsOut, Object input1, Object input2, Object input3, 
                     Object input4)
    {
      return mcr.EvaluateFunction(numArgsOut, "my_mpc", input1, input2, input3, input4);
    }


    /// <summary>
    /// Provides an interface for the my_mpc function in which the input and output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// Define the variables
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("my_mpc", 4, 1, 0)]
    protected void my_mpc(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("my_mpc", numArgsOut, ref argsOut, argsIn, varArgsIn);
    }

    /// <summary>
    /// This method will cause a MATLAB figure window to behave as a modal dialog box.
    /// The method will not return until all the figure windows associated with this
    /// component have been closed.
    /// </summary>
    /// <remarks>
    /// An application should only call this method when required to keep the
    /// MATLAB figure window from disappearing.  Other techniques, such as calling
    /// Console.ReadLine() from the application should be considered where
    /// possible.</remarks>
    ///
    public void WaitForFiguresToDie()
    {
      mcr.WaitForFiguresToDie();
    }



    #endregion Methods

    #region Class Members

    private static MWMCR mcr= null;

    private static Exception ex_= null;

    private bool disposed= false;

    #endregion Class Members
  }
}
