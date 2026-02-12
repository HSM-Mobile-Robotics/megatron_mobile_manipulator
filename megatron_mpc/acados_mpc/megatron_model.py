from acados_template import AcadosModel
from casadi import SX, vertcat, cos, sin


def export_robot_model() -> AcadosModel:

    model_name = 'megatron'

    # CasADi symbols
    # -------------------------------------------------------- #

    # states (f_exp)
    x_q = SX.sym('x')  # position of the robot in x (meters)
    y_q = SX.sym('y')  # position of the robot in y (meters)
    theta_q = SX.sym('theta')  # heading of the robot (radians)
    v_q = SX.sym('v')  # linear velocity of the robot (meters/sec)
    w_q = SX.sym('w')  # angular velocity of the robot (rad/sec)
    x = vertcat(x_q, y_q, theta_q, v_q, w_q)  # state vector matrix

    # controls sent to the robot PID

    v_cmd = SX.sym('v_cmd')  # linear velocity commands (meters/sec)
    w_cmd = SX.sym('w_cmd')  # angular velocity command (meters/sec)
    u = vertcat(v_cmd, w_cmd)  # Control vector matrix

    # Dynamics (continuous-time)
    # -------------------------------------------------------- #

    x_qdot = SX.sym('x_dot')
    y_qdot = SX.sym('y_dot')
    theta_qdot = SX.sym('theta_dot')
    v_qdot = SX.sym('v_dot')
    w_qdot = SX.sym('w_dot')
    x_dot = vertcat(x_qdot, y_qdot, theta_qdot, v_qdot, w_qdot)

    # Robot parameters
    # -------------------------------------------------------- #
    # Need to be calculated from the TF
    v_gain = 1.738  # linear velocity steady state error
    v_time_Constant = -1.809  # linear velocity lag time constant from TF
    w_gain = 0.8597  # angular velocity steady state error
    w_time_Constant = -1.373  # angular velocity lag time constant from TF

    # Dynamics equations
    dx_q = v_q * cos(theta_q)
    dy_q = v_q * sin(theta_q)
    dtheta_q = w_q
    dv_q = (v_time_Constant*v_q) + (v_gain*v_cmd)
    dw_q = (w_time_Constant*w_q) + (w_gain*w_cmd)

    f_explicit = vertcat(dx_q, dy_q, dtheta_q, dv_q, dw_q)

    f_implicit = x_dot - f_explicit

    model = AcadosModel()

    z = []
    p = []

    model.f_impl_expr = f_implicit
    model.f_expl_expr = f_explicit
    model.x = x
    model.xdot = x_dot
    model.u = u
    model.name = model_name
    model.z = z
    model.p = p

    return model
