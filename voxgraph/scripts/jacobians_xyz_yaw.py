#!/usr/bin/env python3
# yapf: disable

from __future__ import print_function
from sympy import simplify, diff, cse
from sympy import Matrix, MatrixSymbol
from sympy import zeros
from sympy import Symbol, symbols
from sympy import cos, sin
from sympy import init_printing, pprint
init_printing(use_unicode=True)

def rotationMatrixFromQuaternion(q_0, q_1, q_2, q_3):
    return Matrix([[q_0**2 + q_1**2 - q_2**2 - q_3**2, 2*q_1*q_2 - 2*q_0*q_3, 2*q_0*q_2 + 2*q_1*q_3],  # pylint: disable=line-too-long
                   [2*q_0*q_3 + 2*q_1*q_2, q_0**2 - q_1**2 + q_2**2 - q_3**2, 2*q_2*q_3 - 2*q_0*q_1],  # pylint: disable=line-too-long
                   [2*q_1*q_3 - 2*q_0*q_2, 2*q_0*q_1 + 2*q_2*q_3, q_0**2 - q_1**2 - q_2**2 + q_3**2]]) # pylint: disable=line-too-long

# Settings
print_jacobians = False
print_jacobians_times_reference_coordinate = True
print_common_subexpression_elimination = False

# Frame and body names
# w: world frame
# o: reference submap frame
# e: reading submap frame
# p: point at which we want to calculate the residual

# Homogeneous coordinate of the point in the reference submap at which we want
# to calculate the residual
o_x_oi, o_y_oi, o_z_oi = symbols('x_i y_i z_i')
o_r_oi = Matrix([o_x_oi, o_y_oi, o_z_oi, 1])

# Translation of the reference submap
x_o, y_o, z_o = symbols('x_o y_o z_o')
w_r_wo = Matrix([x_o, y_o, z_o])

# Translation of the reading submap
x_e, y_e, z_e = symbols('x_e y_e z_e')
w_r_we = Matrix([x_e, y_e, z_e])

# Rotation matrix of the reference submap
theta_o = Symbol('theta_o')
# C_wo = rotationMatrixFromQuaternion(cos(theta_o/2), 0, 0, sin(theta_o/2))
C_wo = Matrix([[cos(theta_o), -sin(theta_o), 0],
               [sin(theta_o), cos(theta_o), 0],
               [0, 0, 1]])

# Rotation matrix of the reading submap
theta_e = Symbol('theta_e')
# C_we = rotationMatrixFromQuaternion(cos(theta_e/2), 0, 0, sin(theta_e/2))
C_we = Matrix([[cos(theta_e), -sin(theta_e), 0],
               [sin(theta_e), cos(theta_e), 0],
               [0, 0, 1]])

# Transform matrix of the reference submap frame to the world frame
T_wo = Matrix(MatrixSymbol('T_wa', 4, 4))
T_wo[:3, :3] = C_wo
T_wo[:3, 3] = w_r_wo
T_wo[3, :3] = zeros(1, 3)
T_wo[3, 3] = 1
# pprint(T_wo)

# NOTE: It would have been nicer to use BlockMatrices instead of assigning
#       through index ranges, but this kept causing block_collapse errors when
#       evaluating the final result

# Transform matrix from the world frame to the reading submap frame
T_ew = Matrix(MatrixSymbol('T_wb_inv', 4, 4))
T_ew[:3, :3] = C_we.T
T_ew[:3, 3] = -C_we.T * w_r_we
T_ew[3, :3] = zeros(1, 3)
T_ew[3, 3] = 1
# pprint(T_ew)

# Transform matrix from the reference submap frame to the reading submap frame
T_eo = simplify(T_ew * T_wo)
# pprint(T_eo)

# Get the derivatives of interest
if print_jacobians:
    print("\nDerivative of T_eo over x_o:")
    pprint(diff(T_eo, x_o))
    print("\nDerivative of T_eo over y_o:")
    pprint(diff(T_eo, y_o))
    print("\nDerivative of T_eo over z_o:")
    pprint(diff(T_eo, z_o))
    print("\nDerivative of T_eo over theta_o:")
    pprint(simplify(diff(T_eo, theta_o)))

    print("\nDerivative of T_eo over x_e:")
    pprint(diff(T_eo, x_e))
    print("\nDerivative of T_eo over y_e:")
    pprint(diff(T_eo, y_e))
    print("\nDerivative of T_eo over z_e:")
    pprint(diff(T_eo, z_e))
    print("\nDerivative of T_eo over theta_e:")
    pprint(simplify(diff(T_eo, theta_e)))

# Get the derivatives over interest times the reference_coordinate
if print_jacobians_times_reference_coordinate:
    print("\nDerivative of T_eo over x_o, times o_r_oi:")
    pprint(diff(T_eo, x_o) * o_r_oi)
    print("\nDerivative of T_eo over y_o, times o_r_oi:")
    pprint(diff(T_eo, y_o) * o_r_oi)
    print("\nDerivative of T_eo over z_o, times o_r_oi:")
    pprint(diff(T_eo, z_o) * o_r_oi)
    print("\nDerivative of T_eo over theta_o, times o_r_oi:")
    pprint(simplify(diff(T_eo, theta_o) * o_r_oi))

    print("\nDerivative of T_eo over x_e, times o_r_oi:")
    pprint(diff(T_eo, x_e) * o_r_oi)
    print("\nDerivative of T_eo over y_e, times o_r_oi:")
    pprint(diff(T_eo, y_e) * o_r_oi)
    print("\nDerivative of T_eo over z_e, times o_r_oi:")
    pprint(diff(T_eo, z_e) * o_r_oi)
    print("\nDerivative of T_eo over theta_e, times o_r_oi:")
    pprint(simplify(diff(T_eo, theta_e) * o_r_oi))

# Get the full derivative matrix after Common Subexpression Elimination
if print_common_subexpression_elimination:
    print("\nDerivative of T_eo over [x_o, y_o, z_o, theta_o]^T, times o_r_oi:")
    pT_eo__pParam_o__roi = Matrix(MatrixSymbol("pTeo_pParam_o__roi", 4, 4))
    pT_eo__pParam_o__roi[:, 0] = diff(T_eo, x_o) * o_r_oi
    pT_eo__pParam_o__roi[:, 1] = diff(T_eo, y_o) * o_r_oi
    pT_eo__pParam_o__roi[:, 2] = diff(T_eo, z_o) * o_r_oi
    pT_eo__pParam_o__roi[:, 3] = simplify(diff(T_eo, theta_o) * o_r_oi)
    pprint(pT_eo__pParam_o__roi)
    print("--> after CSE:")
    pprint(cse(pT_eo__pParam_o__roi))


    print("\nDerivative of T_eo over [x_e, y_e, z_e, theta_e]^T, times o_r_oi:")
    pT_eo__pParam_e__roi = Matrix(MatrixSymbol("pTeo_pParam_e__roi", 4, 4))
    pT_eo__pParam_e__roi[:, 0] = diff(T_eo, x_e) * o_r_oi
    pT_eo__pParam_e__roi[:, 1] = diff(T_eo, y_e) * o_r_oi
    pT_eo__pParam_e__roi[:, 2] = diff(T_eo, z_e) * o_r_oi
    pT_eo__pParam_e__roi[:, 3] = simplify(diff(T_eo, theta_e) * o_r_oi)
    pprint(pT_eo__pParam_e__roi)
    print("--> after CSE:")
    pprint(cse(pT_eo__pParam_e__roi))

    print("\nOr combining everything:")
    pT_eo__pParam_oe__roi = Matrix(MatrixSymbol("pTeo_pParam_oe__roi", 4, 8))
    pT_eo__pParam_oe__roi[:, :4] = pT_eo__pParam_o__roi
    pT_eo__pParam_oe__roi[:, 4:] = pT_eo__pParam_e__roi
    cse_pT_eo__pParam_oe__roi = cse(pT_eo__pParam_oe__roi)
    pprint(cse_pT_eo__pParam_oe__roi[0])
    pprint(cse_pT_eo__pParam_oe__roi[1])
