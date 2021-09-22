

#include "edge_project_aru2uv.h"


namespace g2o {

EdgeProjectARU2UV::EdgeProjectARU2UV() : BaseFixedSizedEdge<2, Vector2, VertexPointXYZ, VertexSE3Expmap, VertexSE3Expmap>() {
    _cam=0;
    resizeParameters(1);
    installParameter(_cam,0);
}

bool EdgeProjectARU2UV::read(std::istream& is) {
    return false;
};

bool EdgeProjectARU2UV::write(std::ostream& os) const {
    return false;
};

void EdgeProjectARU2UV::computeError() {
    const auto mk_obj_p3d = static_cast<const g2o::VertexPointXYZ*>(_vertices[0])->estimate();
    const auto cam_T_pen = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1])->estimate();
    const auto pen_T_face = static_cast<const g2o::VertexSE3Expmap*>(_vertices[2])->estimate();

    const CameraParameters* cam = static_cast<const CameraParameters*>(parameter(0));

    // _error = measurement() - cam->cam_map(cam_T_pen.map(pen_T_face.map(mk_obj_p3d)));
    _error = measurement() ;//- cam->cam_map(cam_T_pen.map(pen_T_face.map(mk_obj_p3d)));
};



}  // namespace g2o
