import pytest
import importlib.util
import inspect

@pytest.mark.dependency(name="exists_module")
def test_exists_module():
  assert importlib.util.find_spec('rdyn_py') != None

def get_module():
  return importlib.import_module('rdyn_py')

@pytest.mark.dependency(name="test_exists_classes", depends=["exists_module"])
@pytest.mark.parametrize('class_name', ['Chain'])
def test_exists_classes(class_name):
  m = get_module()
  list_of_classes = inspect.getmembers(m, inspect.isclass)
  assert [name for name, _ in list_of_classes if name == class_name]


@pytest.mark.dependency(name="test_exists_classes", depends=["exists_module"])
@pytest.mark.parametrize('function_name', ['createChain'])
def test_exists_functions(function_name):
  m = get_module()
  list_of_classes = inspect.getmembers(m, inspect.isroutine)
  assert [name for name, _ in list_of_classes if name == function_name]
