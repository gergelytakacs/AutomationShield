'''Automatically generate C code for a muaompc model predictive controller.

This module is intended to be imported by an instance of any of the muaompc classes.

Provided functions:
write_c_files -- write C code in the current path for the given inputs.

'''
import os
import shutil

import numpy as np

ROWS, COLS = (0, 1)
FLOAT32, FLOAT64, ACCUM, FIP = (-1, -2, 100, 200)
cmpc_dir = 'cmpc/'  # The directory where the c files are to be written

def write_c_files(defines, mtxs, scalars,
                              enums, prefix, numeric, matlab=False,
                              singledir=False):
    '''Write, in the current path, C code from the given inputs.

    This functions loads a set of python/C templates found in the path given by
    prefix, and substitute the variables in the template using the inputs.
    A set of files is generated with fixed problem data and array sizes
    (thought as for embedded platforms), which can be straightforwardly used.

    Inputs:
    defines -- is an iterable, each element contains: 1. a string with the
    definition, 2. a string with the identificator, and 3.
    a string with comment of the define.
    mtxs -- a dictionary with the muaompc matrices to be written as C arrays.
    scalars -- a dictionary with the muaompc scalars be written as C variables.
    enums -- a dictionary with the integer constants to be written as C enums.
    prefix -- the path (relative or absolute) where to find the directory
    named template containing the python/C template files. prefix normally
    points to the muaompc root directory.
    numeric -- an integer indicating the numeric representation of the
    C variables. FLOAT64 is float64_t, FLOAT32 is float32_t, a non negative
    number indicates the number of fixed-point fractional bits, according to
    fractional bits = numeric - fixed_point_type; where fixed_point_type is
    ACCUM or FIP. The FIP implementation is experimental.
     '''
    _copy_fixed_code(prefix, matlab)
    _write_const(defines, mtxs, scalars, enums, prefix)
    _write_mpc_code(prefix, numeric)

    if matlab == True:
        _copy_simulink_code(prefix)
        _write_simulink_config(prefix, enums, numeric)

    if singledir:
        _move_to_singledir()

def _write_vla_c_files(prefix, numeric=FLOAT64):
    """Write a C code that uses variable length arrays.
    No data files are written. The mpc structures need to be filled with data
    by the user (on runtime). This is intended to be used by the interfaces
    to other languages (e.g. Python, Matlab).
    """
    _copy_fixed_code(prefix)
    _write_mpc_code(prefix, numeric, is_embedded=False)

def _write_const(defines, mtxs, scalars, enums, prefix):
    cname = 'mpc_const.c'
    hname = 'mpc_const.h'

    cft = _get_template(prefix, cname)
    hft = _get_template(prefix, hname)

    cfts = cft.format(HoL = _mtx2str(mtxs.HoL),
                      GoL = _mtx2str(mtxs.GoL),
                      Bh_T = _mtx2str(mtxs.Bh_T),
                      E = _mtx2str(mtxs.E),
                      Kx_Ai = _mtx2str(mtxs.Kx_Ai),
                      u_lb = _mtx2str(mtxs.u_lb),
                      u_ub = _mtx2str(mtxs.u_ub),
                      e_lb = _mtx2str(mtxs.e_lb),
                      e_ub = _mtx2str(mtxs.e_ub),
                      Q = _mtx2str(mtxs.Q),
                      R = _mtx2str(mtxs.R),
                      P = _mtx2str(mtxs.P),
                      K = _mtx2str(mtxs.K),
                      Ad = _mtx2str(mtxs.Ad),
                      Bd = _mtx2str(mtxs.Bd),
                      dt = str(scalars.dt),
                      nu = str(scalars.nu),
                      mu = str(scalars.mu),
                      Linv = str(scalars.linv))

    defines_str = _defines2str(defines)
    hfts = hft.format(extra_defines=defines_str,
                      HOR = str(enums.hor),
                      STATES = str(enums.states),
                      INPUTS = str(enums.inputs),
                      MXCONSTRS = str(enums.mxconstrs),
                      HOR_INPUTS = str(enums.hor_inputs),
                      HOR_STATES = str(enums.hor_states),
                      HOR_MXCONSTRS = str(enums.hor_mxconstrs))

    with open(cmpc_dir + cname, 'w') as f:
        f.write(cfts)
    with open(cmpc_dir + 'include/' + hname, 'w') as f:
        f.write(hfts)

def _copy_simulink_code(prefix):
    shutil.copytree(prefix + '/template/simulink', cmpc_dir + '/simulink')

def _write_simulink_config(prefix, enums, numeric):
    tpl = _get_template(prefix, 'simulink_mpcsolve.c')
    tpl = tpl.replace('{STATES}', str(enums.states))
    tpl = tpl.replace('{INPUTS}', str(enums.inputs))
    tpl = tpl.replace('{NUMERIC_DTYPE}',
        str(_get_dtype_for_numeric(numeric)))
    tpl = tpl.replace('{NUMERIC_WORDLENGTH}',
        str(_get_fixpoint_word_size_for_numeric(numeric)))
    tpl = tpl.replace('{NUMERIC_FRACBITS}',
        str(_get_fracbits_for_numeric(numeric)))
    with open(cmpc_dir + 'simulink/mpcsolve.c', 'w') as f:
        f.write(tpl)
        f.close()

def _write_mpc_code(prefix, numeric, is_embedded=True):
    """
    is_embedded -- if True, write C code with fixed array size. Else, write C
    code using variable length arrays. (default True).
    """

    sname = 'mpc_stc.c'
    iname = 'mpc_inc.c'
    rname = 'mpc_ref.c'
    mname = 'mpc.c'
    hname = 'mpc.h'
    bname = 'mpc_base.h'

    sft = _get_template(prefix, sname)
    ift = _get_template(prefix, iname)
    rft = _get_template(prefix, rname)
    mft = _get_template(prefix, mname)
    hft = _get_template(prefix, hname)
    bft = _get_template(prefix, bname)

    real_t_str = _real_t2str(numeric)
    fip_defines_str = _defines2str(_fip_defines(numeric))

    if is_embedded:
        sfts = sft.format(STATES = 'MPC_STATES',
                            HOR_INPUTS = 'MPC_HOR_INPUTS',
                            MXCONSTRS = 'MPC_MXCONSTRS',
                            HOR_MXCONSTRS = 'MPC_HOR_MXCONSTRS',
                            HOR = 'MPC_HOR',
                            STATIC = 'static ',
                            CONST = 'const ',
                            MPC_CONST = '#include "mpc_const.h"')
        ifts = ift.format(HOR = 'MPC_HOR',
                            STATES = 'MPC_STATES',
                            INPUTS = 'MPC_INPUTS',
                            HOR_INPUTS = 'MPC_HOR_INPUTS',
                            STATIC = 'static ',
                            MPC_CONST = '#include "mpc_const.h"')
        rfts = rft.format(HOR = 'MPC_HOR',
                            STATES = 'MPC_STATES',
                            INPUTS = 'MPC_INPUTS',
                            HOR_INPUTS = 'MPC_HOR_INPUTS',
                            HOR_STATES = 'MPC_HOR_STATES',
                            STATIC = 'static ',
                            MPC_CONST = '#include "mpc_const.h"')
        mfts = mft.format(STATES = 'MPC_STATES',
                            INPUTS = 'MPC_INPUTS',
                            HOR_MXCONSTRS = 'MPC_HOR_MXCONSTRS',
                            HOR_STATES = 'MPC_HOR_STATES',
                            HOR_INPUTS = 'MPC_HOR_INPUTS',
                            HOR = 'MPC_HOR',
                            MPC_CONST = '#include "mpc_const.h"',
                            ctl_x_ref = 'extern real_t ctl_x_ref[];\nctl->x_ref = ctl_x_ref;\n',
                            ctl_u_ref = 'extern real_t ctl_u_ref[];\nctl->u_ref = ctl_u_ref;\n')
        hfts = hft.format(MPC_CONST = '#include "mpc_const.h"')
        bfts = bft.format(real_t=real_t_str, defines=fip_defines_str,
                          const='const ')
    else:
        sfts = sft.format(HOR = 'ctl->alm->fgm->HOR',
                            STATES = 'alm->fgm->STATES',
                            MXCONSTRS = 'ctl->alm->MXCONSTRS',
                            HOR_INPUTS = 'alm->fgm->HOR_INPUTS',
                            HOR_MXCONSTRS = 'alm->HOR_MXCONSTRS',
                            STATIC = '',
                            CONST = '',
                            MPC_CONST = '')
        ifts = ift.format(HOR = 'ctl->alm->fgm->HOR',
                            STATES = 'fgm->STATES',
                            INPUTS = 'ctl->alm->fgm->INPUTS',
                            HOR_INPUTS = 'fgm->HOR_INPUTS',
                            STATIC = '',
                            MPC_CONST = '')
        rfts = rft.format(HOR = 'ctl->alm->fgm->HOR',
                            STATES = 'ctl->alm->fgm->STATES',
                            INPUTS = 'ctl->alm->fgm->INPUTS',
                            HOR_INPUTS = 'ctl->alm->fgm->HOR_INPUTS',
                            HOR_STATES = 'ctl->alm->fgm->HOR_STATES',
                            STATIC = '',
                            MPC_CONST = '')
        mfts = mft.format(STATES = 'ctl->alm->fgm->STATES',
                            INPUTS = 'ctl->alm->fgm->INPUTS',
                            HOR_MXCONSTRS = 'ctl->alm->HOR_MXCONSTRS',
                            HOR_STATES = 'ctl->alm->fgm->HOR_STATES',
                            HOR_INPUTS = 'ctl->alm->fgm->HOR_INPUTS',
                            HOR = 'ctl->alm->fgm->HOR',
                            MPC_CONST = '',
                            ctl_x_ref = '',
                            ctl_u_ref = '')
        hfts = hft.format(MPC_CONST = '')
        bfts = bft.format(real_t=real_t_str, defines=fip_defines_str,
                           const='')

    with open(cmpc_dir + sname, 'w') as f:
        f.write(sfts)
    with open(cmpc_dir + iname, 'w') as f:
        f.write(ifts)
    with open(cmpc_dir + rname, 'w') as f:
        f.write(rfts)
    with open(cmpc_dir + mname, 'w') as f:
        f.write(mfts)
    with open(cmpc_dir + 'include/' + hname, 'w') as f:
        f.write(hfts)
    with open(cmpc_dir + 'include/' + bname, 'w') as f:
        f.write(bfts)

def _copy_fixed_code(prefix, matlab=False):
    """Copy all problem-independent code, and create the directory tree.

    This file should be called before any of the other, as it creates the
    required directories.
    """
    subdir = '/template/'

    try:
        shutil.rmtree(cmpc_dir)
    except OSError:
        pass  # OK, directory does not exist
    shutil.copytree(prefix + subdir + cmpc_dir, cmpc_dir)

    if matlab:
        shutil.copytree(prefix + subdir + 'matlab', cmpc_dir + '/matlab')

def _move_to_singledir():
    os.rename('cmpc', 'tmp_cmpc')
    os.mkdir('cmpc')
    for src, subdirs, fnames in os.walk('tmp_cmpc'):
        for fname in fnames:
            fpath = os.path.join(src, fname)
            shutil.copy(fpath, 'cmpc')
    shutil.rmtree('tmp_cmpc', ignore_errors=True)

def _get_template(prefix, name):
    subdir = '/template/'
    f = open(prefix + subdir + 'template_' + name, 'r')
    ft = f.read()
    f.close()
    return ft

# TODO: refactor numeric to state object
def _get_fracbits_for_numeric(numeric):
    if numeric >= FIP:
        return numeric - FIP
    elif numeric >= ACCUM:
        return numeric - ACCUM

def _get_dtype_for_numeric(numeric):
    if numeric >= FIP:
        real_t = 'fixpt'
    elif numeric >= ACCUM:
        real_t = 'fixpt'
    elif numeric is FLOAT32:
        real_t = 'real32_T'
    elif numeric is FLOAT64:
        real_t = 'real_T'

    return real_t

def _get_fixpoint_word_size_for_numeric(numeric):
    return 32

def _fip_defines(numeric):
    defines = []
    if numeric >= FIP:
        fip_fracbits = _get_fracbits_for_numeric(numeric)
        defines.append(['', 'FIP_OPS', 
                    'Compute using fixed point arithmetics (EXPERIMENTAL!).'])
        defines.append([fip_fracbits, 'FRAC_BITS', 
                    'Number of bits for the fixed-point fractional part.'])
    return defines

def _real_t2str(numeric):
    if numeric >= FIP:
        real_t = 'int32_t'
    elif numeric >= ACCUM:
        real_t = 'accum'
    elif numeric is FLOAT32:
        real_t = 'float32_t'
    elif numeric is FLOAT64:
        real_t = 'float64_t'

    return real_t

def _mtx2str(mtx):
    mtx_str = ''
    mtx = np.array(mtx)
    for rows in mtx:
#        f.write('{')
        for elem in rows:
            mtx_str += str(elem)
            mtx_str += ', '
#        f.write('},\n')
        mtx_str += '\n'
    return mtx_str

def _defines2str(dfs):
    df_str = ''
    for df, df_name, comment in dfs:
        df_str += _define2str(df, df_name, comment)
    return df_str

def _define2str(df, df_name, comment):
    ctype = ''.join(['#define ', df_name, ' '])
    declaration = ctype + str(df) + ' /* ' + comment + ' */ \n'
    return declaration

