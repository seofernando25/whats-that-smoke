"""Reference motor conversions and rigid rubber-contact material parameters."""
from pathlib import Path
import json
import math

HERE = Path(__file__).resolve().parent

def load_profile():
    profile=json.loads((HERE/'motor_profile.json').read_text())
    ratio=profile['voltage_v_assumed']/profile['reference_voltage_v']
    profile['stall_nm']=profile['stall_torque_kgf_cm_reference']*.0980665*ratio
    profile['omega0']=profile['no_load_rpm_reference']*2*math.pi/60*ratio
    return profile

def add_rubber_surface(collision, node, profile):
    surface=node(collision,'surface')
    friction=node(surface,'friction')
    ode=node(friction,'ode')
    node(ode,'mu',profile['tire_mu_longitudinal_assumed'])
    node(ode,'mu2',profile['tire_mu_lateral_assumed'])
    node(ode,'fdir1','0 0 1')  # Cylinder-local Z maps to the wheel/link Y axle.
    node(ode,'slip1',profile['tire_slip_lateral_m_per_s_n_assumed'])
    node(ode,'slip2',profile['tire_slip_longitudinal_m_per_s_n_assumed'])
    # Equal coefficients are intentionally isotropic; ordinary rubber wheels
    # scrub sideways during a skid-steer pivot, not frictionless caster wheels.
    bounce=node(surface,'bounce')
    node(bounce,'restitution_coefficient',profile['tire_restitution_assumed'])
    node(bounce,'threshold',.1)

def add_visual_material(mat, node, color, profile):
    metal=node(node(mat,'pbr'),'metal')
    node(metal,'metalness',.7 if color in ('silver','gold') else 0)
    node(metal,'roughness',profile['rubber_roughness_visual'] if color=='rubber' else .4)
