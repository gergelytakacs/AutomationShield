# This script is used to generate files containing C code for FloatShield Arduino IDE MPC example
import muaompc  # Import muaompc module (http://ifatwww.et.uni-magdeburg.de/syst/muAO-MPC/)

# Note: the "numpy" module is required to run this script
mpc = muaompc.ltidt.setup_mpc_problem('model_parameters')  # Load model parameters from the specified file
mpc.generate_c_files(numeric='float32', singledir=True)  # Generate respective C files, use float type for its variables