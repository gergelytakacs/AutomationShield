import numpy as np
import scipy.linalg as la

TOL = 1e-9

def controllable_states(A, B, tol = TOL):
	""" Returns the number of controllable states from a (A,B) UCSF 

	Arguments:
	A -- System matrix as numpy.matrix
	B -- Input matrix as numpy.matrix
	
	Optional arguments:
	tol -- Numerical tolerance (default 1e-9)
	"""
	n, m = B.shape
	U, S, V = la.svd(B)
	n_c = np.sum(S > tol)
	if n_c == 0: return 0

	while np.sum(abs(A[n_c:n, 0:n_c])) > tol: n_c += 1
	return n_c

class _Impl_Base(object):
	""" Base implementation for different algorithm approaches """
	
	def __init__(self, tol):
		""" Initializes a new implementation

		Arguments:
		tol -- Numerical tolerance"""
		self.tol = tol

	def input_transformation(self, B):
		""" Computes the data needed for the input transformation step (see PDF)

		Arguments:
		B -- Input matrix as numpy.matrix

		Returns a tuple containing the transformation matrix and the number of input controllable states
		"""
		raise NotImplementedError()

	def state_transformation(self, Ai):
		""" Computes the data needed for the state transformation steps (see PDF)

		Arguments:
		Ai -- (Partial) state matrix as numpy.matrix

		Returns a tuple containing the transformation matrix for the partial step and the number of revealed controllable states
		""" 
		raise NotImplementedError()

class _Impl_SVD(_Impl_Base):
	""" Implementation of UCSF algorithm using SVD.

	Standard implementation of UCSF transformation. The numerical values differ from the implementation in Ocatave (see QR implementation)."""

	def input_transformation(self, B):
		""" See _Impl_Base.input_transformation """
		U, S, V = la.svd(B)
		return U.T, sum(S > self.tol)

	def state_transformation(self, Ai):
		""" See _Impl_Base.state_transformation """
		U, S, V = la.svd(Ai)
		return la.block_diag(V.T, U.T), sum(S > self.tol)

class _Impl_QR(_Impl_Base):
	""" Implementation of UCSF algorithm using QR factorization.

	This implementation produces the same output like ctrbf in Octave."""

	def input_transformation(self, B):
		""" See _Impl_Base.input_transformation """
		Q, R = la.qr(B)
		U, S, V = la.svd(R) # TODO: Use rank-revealing QR factorization
		return Q.T, sum(S > self.tol)

	def state_transformation(self, Ai):
		""" See _Impl_Base.state_transformation """
		Q, R = la.qr(Ai)
		U, S, V = la.svd(R) # TODO: Use rank-revealing QR factorization
		return la.block_diag(np.eye(R.shape[1]), Q.T), sum(S > self.tol)

def ucsf(A, B, C, tol = TOL, impl = 'svd'):
	""" 
	Computes the Upper Controllability Staircase Form for a system (A,B,C)

	The transformation can be written as Ac = T * A * T.T and uses either 
	an implementation based un a Singular Value Decomposition, which is the 
	standard algorithm.
	Alternatively an algorithm based on QR factorization can be used.

	Both algorithms will reveal the same system structure, but will differ in
	the numerical values. The QR implementation produces the exact same values
	like the implementation in Octave.

	Arguments:
	A -- System matrix as numpy.matrix
	B -- Input matrix as numpy.matrix
	C -- Output matrix as numpy.matrix

	Optional arguments:
	tol -- Numerical tolerance (default 1e-9)
	impl -- Implementation of the algorithm (either "svd" or "qr", default is SVD)

	Returns a tuple (A, B, C, T) with:
	A, B, C -- Transfomed matrices as numpy.matrix
	T -- Transformation matrix as numpy.matrix
	"""
	if impl == 'svd': impl = _Impl_SVD(tol)
	elif impl == 'qr': impl = _Impl_QR(tol)
	else: raise ValueError('Available implementations: svd, qr')

	nc = []
	n, m = B.shape

	# Input transformation
	T, ni = impl.input_transformation(B)

	A = T * A * T.T
	B = T * B

	nc.append(ni)

	# System transformation
	# First iteration
	Ai = A[sum(nc):n, 0:sum(nc)]
	if abs(np.sum(Ai)) > tol:
		Ti, ni = impl.state_transformation(Ai)
		nc.append(ni)

		A = Ti * A * Ti.T
		B = Ti * B
		T = Ti * T

		Ai = A[sum(nc):n, sum(nc[0:len(nc)-1]):sum(nc)]

		# Further iterations
		i = 2
		while abs(np.sum(Ai)) > tol:
			Ti, ni = impl.state_transformation(Ai)
			nc.append(ni)

			Ti = la.block_diag(np.eye(sum(nc[0:i-1])), Ti)
			A = Ti * A * Ti.T
			B = Ti * B
			T = Ti * T

			Ai = A[sum(nc):n, sum(nc[0:len(nc)-1]):sum(nc)]
			i = i + 1			

	return A, B, C * T.T, T
