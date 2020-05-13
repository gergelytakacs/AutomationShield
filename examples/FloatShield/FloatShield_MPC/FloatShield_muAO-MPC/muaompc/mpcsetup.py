r"""
.. default-role:: math

The plant to be controlled is described by `x^+ = A_d x + B_d u`, where 
`x \in \mathcal{C}_x \subseteq \mathbb{R}^n`, and `u \in \mathcal{C}_u \subset \mathbb{R}^m` are the 
current state and input vector, respectively. The the state at the next
sampling time is denoted by `x^+`.
The discrete-time system and input matrices are denoted as 
`A_d` and `B_d`, respectively. 

The MPC setup is as follows:

.. math::
   \underset{u}{\text{minimize}} & \;\; 
   \frac{1}{2} \sum\limits_{j=0}^{N-1} ((x_j - x\_ref_j)^T Q (x_j - x\_ref_j) + \\
   & (u_j - u\_ref_j)^T R (u_j - u\_ref_j)) + \\
   & \frac{1}{2} (x_{N} - x\_ref_N)^{T} P (x_{N} - x\_ref_N) \\
   \text{subject to} 
   & \;\; x_{j+1}=A_d x_j + B_d u_j, \;\; j = 0, \cdots, N-1 \\
   & \;\; u\_lb \leq u_j \leq u\_ub, \;\; j = 0, \cdots, N-1   \\
   & \;\; e\_lb \leq K_{x} x_j + K_{u} u_j \leq e\_ub, \;\; j  = 0, \cdots, N-1 \\
   & \;\; f\_lb \leq F x_N \leq f\_ub \\
   & \;\; x_0 = x \\


where the integer `N \geq 2` is the prediction horizon.  The symmetric matrices
`Q`, `R`, and `P` are the state, input, and final state weighting matrices, 
respectively.
The inputs are constrained by a box set, defined by the lower
and upper bound `u\_lb` and `u\_ub`, respectively. 
Furthermore, the state and input vectors are also constrained (mixed
constraints), using `K_{x}` and 
`K_{u}`, and delimited by the lower and upper bounds `e\_lb` and `e\_ub \in
\mathbb{R}^q`, respectively.  Additionally, the terminal state needs 
to satisfy some constraints defined by `f\_lb`, `f\_ub \in \mathbb{R}^r`, 
and the matrix `F`. 

The sequences `x\_ref` and `u\_ref` denote the state and input references, 
respectively.  They are specified online. By default, the reference
is the origin. 

"""
