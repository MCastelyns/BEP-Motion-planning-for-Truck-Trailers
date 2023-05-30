MyMPC MATLAB .NET Assembly (.NET Component)

1. Prerequisites for Deployment 

Verify that MATLAB Runtime(R2023a) is installed.   
If not, you can run the MATLAB Runtime installer.
To find its location, enter
  
    >>mcrinstaller
      
at the MATLAB prompt.
NOTE: You will need administrator rights to run the MATLAB Runtime installer. 

Alternatively, download and install the Windows version of the MATLAB Runtime for R2023a 
from the following link on the MathWorks website:

    https://www.mathworks.com/products/compiler/mcr/index.html
   
For more information about the MATLAB Runtime and the MATLAB Runtime installer, see 
"Distribute Applications" in the MATLAB Compiler SDK documentation  
in the MathWorks Documentation Center.

2. Files to Deploy and Package

-MyMPC.dll
   -contains the generated component using MWArray API. 
-MyMPCNative.dll
   -contains the generated component using native API.
-This readme file
    Note: if end users are unable to download the MATLAB Runtime using the
    instructions in the previous section, include it when building your 
    component by clicking the "Runtime included in package" link in the
    Deployment Tool.


Auto-generated Documentation Templates:

MWArray.xml - This file contains the code comments for the MWArray data conversion 
              classes and their methods. This file can be found in either the component 
              distrib directory or in
              <mcr_root>*\toolbox\dotnetbuilder\bin\win64\v4.0

MyMPC_overview.html - HTML overview documentation file for the generated component. It 
                      contains the requirements for accessing the component and for 
                      generating arguments using the MWArray class hierarchy.

MyMPC.xml - This file contains the code comments for the MyMPC component classes and 
                      methods. Using a third party documentation tool, this file can be 
                      combined with either or both of the previous files to generate 
                      online documentation for the MyMPC component.

3. Resources

To learn more about:               See:
===================================================================
MWArray classes                    <matlab_root>*\help\toolbox\
                                   dotnetbuilder\MWArrayAPI\
                                   MWArrayAPI.chm  
Examples of .NET Web Applications  Web Deployment in the MATLAB   
                                   .NET Assembly documentation in the  
                                   MathWorks Documentation Center


4. Definitions

For information on deployment terminology, go to
https://www.mathworks.com/help and select MATLAB Compiler >
Getting Started > About Application Deployment >
Deployment Product Terms in the MathWorks Documentation
Center.

* NOTE: <mcr_root> is the directory where the MATLAB Runtime is installed on the target 
        machine.
        <matlab_root> is the directory where MATLAB is installed on the target machine.

