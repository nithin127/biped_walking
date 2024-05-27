# Author: Nithin Vasishta

import numpy as np
import qpSWIFT as qp

class QPSolver():
	def __init__(self, L_i, K_i, horizon: int = 10, decay=0.99):

		self.horizon = horizon
		
		n = L_i.shape[0]
		m = K_i.shape[0]
		self.L = np.zeros((n*horizon, n*horizon))
		self.K = np.zeros((m*horizon, m*horizon))

		for i in range(horizon):
			self.L[i*n:(i+1)*n, i*n:(i+1)*n] = L_i*(decay**i)
			self.K[i*m:(i+1)*m, i*m:(i+1)*m] = K_i*(decay**i)

		# self.opts = {'MAXITER':30,'VERBOSE':1,'OUTPUT':2}
		self.opts = {'MAXITER':30,'VERBOSE':0,'OUTPUT':2}


	def get_qp_matrices(self, A_mat, B_mat):
		
		A_i = np.array(A_mat).copy()
		A_qp = np.array(A_mat).copy()
		B_i = np.array(B_mat).copy()
		B_qp = np.array(B_mat).copy()

		for i in range(self.horizon-1):
			B_i = np.hstack((A_i@B_mat, B_i))
			A_i = A_i@A_mat
			A_qp = np.vstack((A_qp, A_i))
			B_qp = np.hstack((B_qp, np.zeros((B_qp.shape[0], B_mat.shape[1])))) # To match dimensions
			B_qp = np.vstack((B_qp, B_i))

		return A_qp, B_qp


	def solve_qp(self, A_mat, B_mat, x_init = None, x_ref = None, 
				force_contraints = (None, None)):
		
		A_qp, B_qp = self.get_qp_matrices(A_mat, B_mat)

		# Cost function
		H = 2*(B_qp.T@self.L@B_qp + self.K)
		g = 2*B_qp.T@self.L@(A_qp@x_init - x_ref)
		
		# Inequality constraints
		c, C = force_contraints
		
		res = qp.run(g, c, H, C, opts=self.opts)

		return res['sol']
		