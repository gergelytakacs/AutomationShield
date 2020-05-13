import muaompc

mpc = muaompc.ltidt.setup_mpc_problem("sys_motor")
mpc.generate_c_files()

