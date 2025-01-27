#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <rdyn_core/primitives.h>

namespace py = pybind11;

PYBIND11_MODULE(rdyn_py, m) {
  py::class_<rdyn::Chain, std::shared_ptr<rdyn::Chain>>(m, "Chain")
    .def(py::init<>())
    .def("getRegressor", &rdyn::Chain::getRegressor,
      py::arg("q"),
      py::arg("Dq"),
      py::arg("DDq")
    )
    .def("getJointTorque", [](rdyn::Chain& self,
                              const Eigen::VectorXd& q,
                              const Eigen::VectorXd& Dq,
                              const Eigen::VectorXd& DDq) -> Eigen::VectorXd
      {
        return self.getJointTorque(q, Dq, DDq);
      },
      py::arg("q"),
      py::arg("Dq"),
      py::arg("DDq")
    )
    .def("getMoveableJointNames", &rdyn::Chain::getMoveableJointNames)
    .def("getLinksNumber", &rdyn::Chain::getLinksNumber)
    .def("getJointsNumber", &rdyn::Chain::getJointsNumber)
    .def("getActiveJointsNumber", &rdyn::Chain::getActiveJointsNumber)
    .def("getActiveJointsName", &rdyn::Chain::getActiveJointsName)
    .def("getLinksName", &rdyn::Chain::getLinksName)
    .def("getNominalParameters", &rdyn::Chain::getNominalParameters);

  m.def("createChain", [](const std::string& file,
                               const std::string& base_frame,
                               const std::string& tool_frame,
                               const Eigen::Vector3d& gravity) -> rdyn::ChainPtr
    {
      return rdyn::createChain(file, base_frame, tool_frame, gravity);
    },
    py::arg("urdf_string"),
    py::arg("base_frame"),
    py::arg("tool_frame"),
    py::arg("gravity") = Eigen::Vector3d({0, 0, -9.806}) // Required?
  );
}
