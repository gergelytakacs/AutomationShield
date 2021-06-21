# MPC problem definition file for muaOMPC in Python
# Visit www.automationshield.com for more information.
# ========================================================

import muaompc 	 # Import muaompc module

# Note: the "numpy" module is required to run this script
mpc = muaompc.ltidt.setup_mpc_problem('model_parameters')  # Load model parameters from the specified file
mpc.generate_c_files(numeric='float32', singledir=True)  # Generate respective C files, use float type for its variables