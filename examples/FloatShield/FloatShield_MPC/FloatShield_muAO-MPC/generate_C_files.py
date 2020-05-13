# This script is used to generate files containing C code for FloatShield Arduino IDE MPC example
import muaompc  # Import muaompc module (http://ifatwww.et.uni-magdeburg.de/syst/muAO-MPC/)
import os   # Import os module required for rearranging the compiled files

# Note: the "numpy" module is required to run this script
mpc = muaompc.ltidt.setup_mpc_problem('model_parameters')  # Load model parameters from the specified file
mpc.generate_c_files(numeric='float32')  # Generate respective C files, use float type for its variables

# An auxiliary routine to rearrange the compiled files
list_of_headers = os.listdir('cmpc/include')
for header in list_of_headers:
    os.replace('cmpc/include/' + header, 'cmpc/' + header)
os.remove('cmpc/Makefile')
os.rmdir('cmpc/include')
