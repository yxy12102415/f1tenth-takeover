"""Generate a native C acados joint dynamic MPC. Build-time Python only."""
import os
from pathlib import Path
import numpy as np
import casadi as ca
import future_fstrings
future_fstrings.register()
from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver

ROOT = Path(os.environ.get('TAKEOVER_CODEGEN_DIR', Path(__file__).resolve().parent)).resolve()
ROOT.mkdir(parents=True, exist_ok=True)
PROFILE = os.environ.get('TAKEOVER_MODEL_PROFILE', 'v8')
if PROFILE not in ('v8', 'official_gym', 'official_gym_native'):
    raise ValueError('Unknown vehicle model profile')
os.chdir(ROOT)
model = AcadosModel()
model.name = 'joint_vehicle'
# Pose, longitudinal/lateral velocity, yaw rate, previously applied a/delta.
x = ca.SX.sym('x', 10)
u = ca.SX.sym('u', 2)
p = ca.SX.sym('p', 5)  # reference x/y/yaw, speed target, corridor half-width
dt = .05
px, py, yaw, v, vy, rate, old_a, old_delta, v_command, servo = ca.vertsplit(x)
a, delta = ca.vertsplit(u)
new_command = v_command+dt*a
steering_tau = .02 if PROFILE != 'v8' else .20
steering_limit = .4189 if PROFILE == 'official_gym_native' else .269
acceleration_limit = 3.0 if PROFILE == 'official_gym_native' else 1.0
deceleration_limit = 3.0 if PROFILE == 'official_gym_native' else 0.8
new_servo = servo+dt/(steering_tau+dt)*(delta-servo)
safe_v = ca.fmax(v, .35)
m, iz, lf, lr, cf, cr = 3.74, .04712, .12, .13, 90., 96.
if PROFILE != 'v8':
    lf, lr = .15875, .17145
    cf = 1.0489*m*9.81*lr/(lf+lr)*4.718
    cr = 1.0489*m*9.81*lf/(lf+lr)*5.4562
A = ca.vertcat(ca.horzcat(-(cf+cr)/(m*safe_v), -safe_v-(cf*lf-cr*lr)/(m*safe_v)),
               ca.horzcat(-(cf*lf-cr*lr)/(iz*safe_v), -(cf*lf*lf+cr*lr*lr)/(iz*safe_v)))
b = ca.vertcat(cf/m, lf*cf/iz)
# Backward Euler avoids stiff-tire explicit integration at 20 Hz.
z = ca.solve(ca.SX.eye(2)-dt*A, ca.vertcat(vy, rate)+dt*b*new_servo)
blend = ca.fmin(1., ca.fmax(0., (v-.25)/.30))
blend = blend*blend*(3.-2.*blend)
longitudinal_tau = ca.if_else(new_command>=v,20./95.1,5./95.1) if PROFILE != 'v8' else .15
new_v = v+dt/(longitudinal_tau+dt)*(new_command-v)
new_vy = blend*z[0]
new_rate = blend*z[1]+(1-blend)*new_v/(lf+lr)*ca.tan(new_servo)
mid_yaw = yaw+.5*dt*new_rate
model.x, model.u, model.p = x, u, p
model.disc_dyn_expr = ca.vertcat(
    px+dt*(.5*(v+new_v)*ca.cos(mid_yaw)-new_vy*ca.sin(mid_yaw)),
    py+dt*(.5*(v+new_v)*ca.sin(mid_yaw)+new_vy*ca.cos(mid_yaw)),
    yaw+dt*new_rate, new_v, new_vy, new_rate, a, delta, new_command, new_servo)
dx, dy = px-p[0], py-p[1]
along = ca.cos(p[2])*dx+ca.sin(p[2])*dy
cross = -ca.sin(p[2])*dx+ca.cos(p[2])*dy
heading = yaw-p[2]
model.cost_y_expr = ca.vertcat(along, cross, heading, v-p[3], vy, rate,
                              a, delta, a-old_a, delta-old_delta)
model.cost_y_expr_e = ca.vertcat(along, cross, heading, v-p[3], vy, rate)
model.con_h_expr = ca.vertcat(cross/p[4], heading, v*rate, a-old_a, delta-old_delta)
model.con_h_expr_0 = model.con_h_expr
model.con_h_expr_e = ca.vertcat(cross/p[4], heading, v*rate)
ocp = AcadosOcp()
ocp.model = model
ocp.dims.N = 24
ocp.parameter_values = np.array([0., 0., 0., 1.0, .20])
ocp.cost.cost_type = ocp.cost.cost_type_e = 'NONLINEAR_LS'
ocp.cost.W = np.diag([3., 160., 140., 18., 1., .5, .3, 1.2, 2., 180.])
ocp.cost.W_e = np.diag([5., 240., 210., 30., 1., .5])
ocp.cost.yref = np.zeros(10)
ocp.cost.yref_e = np.zeros(6)
ocp.constraints.idxbu = np.array([0, 1])
ocp.constraints.lbu = np.array([-deceleration_limit, -steering_limit])
ocp.constraints.ubu = np.array([acceleration_limit, steering_limit])
ocp.constraints.idxbx = np.array([3, 4, 5, 8])
ocp.constraints.lbx = np.array([0., -2.5, -8., 0.])
ocp.constraints.ubx = np.array([2.3, 2.5, 8., 2.0])
ocp.constraints.idxbx_e = ocp.constraints.idxbx.copy()
ocp.constraints.lbx_e = ocp.constraints.lbx.copy()
ocp.constraints.ubx_e = ocp.constraints.ubx.copy()
ocp.constraints.lh = np.array([-1., -.7, -1.65, -.15, -.04])
ocp.constraints.uh = -ocp.constraints.lh
ocp.constraints.lh_0 = np.array([-1e3, -.7, -10., -.20, -.05])
ocp.constraints.uh_0 = -ocp.constraints.lh_0
ocp.constraints.lh_e = np.array([-1., -.7, -1.65])
ocp.constraints.uh_e = -ocp.constraints.lh_e
ocp.constraints.x0 = np.zeros(10)
ocp.solver_options.tf = 1.2
ocp.solver_options.integrator_type = 'DISCRETE'
ocp.solver_options.nlp_solver_type = 'SQP_RTI'
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.qp_solver_cond_N = 6
ocp.solver_options.qp_solver_iter_max = 50
ocp.solver_options.qp_solver_tol_stat = 1e-7
ocp.solver_options.qp_solver_tol_eq = 1e-7
ocp.solver_options.qp_solver_tol_ineq = 1e-7
ocp.solver_options.qp_solver_tol_comp = 1e-7
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.levenberg_marquardt = 1e-4
ocp.solver_options.print_level = 0
ocp.code_export_directory = str(ROOT/'generated_joint')
AcadosOcpSolver.generate(ocp, json_file=str(ROOT/'joint_solver.json'))
if os.environ.get('TAKEOVER_GENERATE_ONLY') != '1':
    AcadosOcpSolver.build(ocp.code_export_directory, with_cython=False)
print('Generated native joint dynamics/control solver')
