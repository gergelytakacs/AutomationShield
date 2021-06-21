
/*
  Autogenerated C-mex file for evaluation of explicit controllers.

  This file is to be compiled under Matlab using the syntax

   mex -largeArrayDims ectrl_mex.c

  Usage in Matlab:

   u = ectrl_mex(x)

  where x is the vector of double for which to evaluate PWA/PWQ function.


  Copyright 2013 by Martin Herceg, Automatic Control Laboratory,
  ETH Zurich, herceg@control.ee.ethz.ch


  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/ 


/* Generated on 16-Mar-2021 10:52:26 by MPT @version@ */ 




#include <math.h>
#include "mex.h"
#include "ectrl.c"

#ifndef MPT_RANGE
#define MPT_RANGE 1
#endif


void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] )

{ 
    int j;
    double *xin, *uout;
    char msg[70];
    unsigned long region;

    mwSize M,N,D; 

    /* Check for proper number of arguments */

    if (nrhs != 1) { 
	    mexErrMsgTxt("One input arguments required."); 
    } else if (nlhs > 1) {
	    mexErrMsgTxt("Too many output arguments."); 
    } 
    if (mxIsEmpty(prhs[0]))
        mexErrMsgTxt("The argument must not be empty.");
	if ( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) )
        mexErrMsgTxt("The argument must be real and of type DOUBLE.");
    if ( mxGetNumberOfDimensions(prhs[0])!=2 )
		mexErrMsgTxt("The argument must be a vector"); 

    M = mxGetM(prhs[0]);
    N = mxGetN(prhs[0]);
    if ( (M!=1) && (N!=1) )
		mexErrMsgTxt("The argument must be a vector.");

    /* dimension */
    D = mxGetNumberOfElements(prhs[0]);
    if (D!=MPT_DOMAIN) {
        sprintf(msg, "The size of the input argument must be %d x 1.",MPT_DOMAIN);
        mexErrMsgTxt(msg);
    }

    /* get and verify input data */
    xin = mxGetPr(prhs[0]);   
    for (j=0; j<D; j++) {
        if ( mxIsNaN(xin[j]) || mxIsInf(xin[j]) )
            mexErrMsgTxt("No 'NaN' or 'Inf' terms are allowed.");
    }

    /* create a matrix for the output */ 
    plhs[0] = mxCreateDoubleMatrix( MPT_RANGE, 1, mxREAL);    
    uout = mxGetPr(plhs[0]);


    /* Do the evaluation in a subroutine */
    region = ectrl(xin, uout);


    return;

}
