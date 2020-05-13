import muaompc 

mpc = muaompc.ltidt.setup_mpc_problem("sys_aircraft")
mpc.generate_c_files()

