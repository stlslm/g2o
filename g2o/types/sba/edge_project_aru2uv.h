#ifndef G2O_SBA_EDGEPROJECTARU2UV_H
#define G2O_SBA_EDGEPROJECTARU2UV_H

// #include "g2o/core/base_variable_sized_edge.h"
#include "g2o/core/base_fixed_sized_edge.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o_types_sba_api.h"
#include "parameter_cameraparameters.h"
#include "vertex_se3_expmap.h"

namespace g2o {

class G2O_TYPES_SBA_API EdgeProjectARU2UV 
    : public BaseFixedSizedEdge<1, Vector2, VertexPointXYZ, VertexSE3Expmap, VertexSE3Expmap> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectARU2UV();

    bool read(std::istream& is);
    bool write(std::ostream& os) const;
    void computeError();

    public:
        CameraParameters* _cam;  
};

}

#endif
