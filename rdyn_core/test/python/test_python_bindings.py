import pytest
import rdyn_py
import os
import inspect
import pandas
import numpy as np

def get_urdf():
  urdf_string = None
  with open(os.path.join('..', 'ur10.urdf'), 'rt') as f:
    urdf_string = f.read()

  return urdf_string

def test_create_chain_from_string():
  urdf_string = get_urdf()
  chain = rdyn_py.createChain(urdf_string, 'base_link', 'tool0')
  assert chain
  assert inspect.ismethod(chain.getRegressor)

def test_regressor():
  urdf_string = get_urdf()
  chain = rdyn_py.createChain(urdf_string, 'base_link', 'tool0')
  pds = pandas.read_csv(filepath_or_buffer='example_trj.csv', delimiter=';', decimal=',')
  q = pds.loc[1, ['pos1_actual [rad]','pos2_actual [rad]','pos3_actual [rad]','pos4_actual [rad]','pos5_actual [rad]','pos6_actual [rad]']].to_numpy()
  Dq = pds.loc[1, ['speed1_actual [rad/sec]','speed2_actual [rad/sec]','speed3_actual [rad/sec]','speed4_actual [rad/sec]','speed5_actual [rad/sec]','speed6_actual [rad/sec]']].to_numpy()
  DDq = pds.loc[1, ['a1_target [rad/sec^2]','a2_target [rad/sec^2]','a3_target [rad/sec^2]','a4_target [rad/sec^2]','a5_target [rad/sec^2]','a6_target [rad/sec^2]']].to_numpy()
  assert not np.isnan(chain.getRegressor(q, Dq, DDq)).any()
