// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <stdint.h>

#include <iostream>
#include <fstream>
#include <assert.h>     /* assert */

// #include <experimental/filesystem>
#include <boost/filesystem.hpp>

#include <string>

#include <unordered_set>

#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/io_helper.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#if defined G2O_HAVE_CHOLMOD
G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
#else
G2O_USE_OPTIMIZATION_LIBRARY(eigen);
#endif

G2O_USE_OPTIMIZATION_LIBRARY(dense);

using namespace Eigen;
using namespace std;
using namespace cv;

// namespace fs = std::experimental::filesystem;
namespace bfs = boost::filesystem;

#define MAXBUFSIZE ((int)1e6)

class Sample {
 public:
  static int uniform(int from, int to) { return static_cast<int>(g2o::Sampler::uniformRand(from, to)); }
};


int read_rvec_tvec_txtfile(string file, vector<g2o::SE3Quat>& cam_T_balls) {
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    if (bfs::exists(bfs::path(file))) {
        std::cout << "Opening file " << file << "..." << std::endl;
    } else {
        std::cout << "File " << file << " does not exist!" << std::endl;
        return -1;
    }

    ifstream infile;
    infile.open(file);
    cam_T_balls.clear();
    while (! infile.eof())
    {
        string line;
        getline(infile, line);
        std::cout << "line " << cam_T_balls.size() << ": " << line << std::endl;

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];
        if (temp_cols<=1) {
            std::cout << "skipping line..." << std::endl;
            continue;
        }            
        int idx= cols*rows;
        Vector3d trans(buff[idx+3], buff[idx+4], buff[idx+5]);
        Mat rv3 = (Mat_<double>(1,3) << buff[idx+0], buff[idx+1], buff[idx+2]);
        Mat R3;
        Rodrigues(rv3, R3);
        Matrix3d R3_;
        cv2eigen(R3, R3_);
        Eigen::Quaterniond q(R3_);
        g2o::SE3Quat pose(q,trans);
        // std::cout << "pose:\n" << pose.toVector() << std::endl;
        // std::cout << "pose: " << q;
        // std::cout << "; " << trans << std::endl;
        cam_T_balls.push_back(pose);

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
    }
    std::cout << "Total size: " << cam_T_balls.size() << std::endl;
    infile.close();
    return true;
}

bool read_aruco_corner_measurements(string folder, vector<vector<Vector2d>>& aru_im_corners) {
    aru_im_corners.clear();

    if (bfs::is_directory(bfs::path(folder.c_str()))) {
        std::set<bfs::path> sort_by_name;
        for (auto&& x : bfs::directory_iterator(bfs::path(folder))) {
            sort_by_name.insert(x.path());
        }
        
        for (auto &x : sort_by_name) {
            std::size_t found = x.filename().string().find("aruco_image_corners_");
            if (found != std::string::npos) {
                vector<Vector2d> frm_crns;

                cout << "parsing file " << x.string() << "...\n";
                ifstream infile;
                infile.open(x.string());
                while (!infile.eof())
                {
                    string line;
                    getline(infile, line);

                    stringstream stream(line);
                    // int cnt = 0;
                    // while(! stream.eof()){
                    //     stream >> buff[cnt];
                    //     cnt++;
                    // }
                    // assert(cnt==1);
                    // int id = buff[0];

                    Vector2d crns;
                    g2o::internal::readVector(stream, crns);
                    frm_crns.push_back(crns);
                }
                infile.close();

                aru_im_corners.push_back(frm_crns);
            }

        }
    } else {
        cout << folder << " is not a dir\n";
        return false;
    }
    std::cout << "Total size: " << aru_im_corners.size() << std::endl;

    return true;
}

bool read_aruco_ids(string folder, vector<vector<int>>& aru_ids) {
    double buff[MAXBUFSIZE];

    aru_ids.clear();

    // list all files in the folder
    std::cout << "directory_iterator:\n";
    if (bfs::is_directory(bfs::path(folder.c_str()))) {
        std::set<bfs::path> sort_by_name;
        for (auto&& x : bfs::directory_iterator(bfs::path(folder))) {
            sort_by_name.insert(x.path());            
        }

    for (auto &x : sort_by_name) {
        std::size_t found = x.filename().string().find("detected_ids_");
        if (found!=std::string::npos) {

          vector<int> frm_ids;

          cout << "parsing file " << x.string() << "...\n";
          ifstream infile;
          infile.open(x.string());
          while (!infile.eof())
          {
            string line;
            getline(infile, line);

            stringstream stream(line);
            int cnt = 0;
            while(! stream.eof()){
              stream >> buff[cnt];
              cnt++;
            }
            assert(cnt==1);
            int id = buff[0];
            frm_ids.push_back(id);
          }
          infile.close();

          aru_ids.push_back(frm_ids);
        }
      }
    } else {
      std::cout << folder << " is not a dir\n" <<std::endl;
      return false;
    }

    std::cout << "Total size: " << aru_ids.size() << std::endl;
    
    return true;
}


int main(int argc, const char* argv[]){
  if (argc<2)
  {
    cout << endl;
    cout << "Please type: " << endl;
    cout << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] [STRUCTURE_ONLY] [DENSE]" << endl;
    cout << endl;
    cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << endl;
    cout << "OUTLIER_RATIO: probability of spuroius observation  (default: 0.0)" << endl;
    cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)" << endl;
    cout << "STRUCTURE_ONLY: performe structure-only BA to get better point initializations (0 or 1; default: 0==false)" << endl;
    cout << "DENSE: Use dense solver (0 or 1; default: 0==false)" << endl;
    cout << endl;
    cout << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to 1==true." << endl;
    cout << endl;
    exit(0);
  }

  double PIXEL_NOISE = atof(argv[1]);
  double OUTLIER_RATIO = 0.0;

  if (argc>2)  {
    OUTLIER_RATIO = atof(argv[2]);
  }

  bool ROBUST_KERNEL = false;
  if (argc>3){
    ROBUST_KERNEL = atoi(argv[3]) != 0;
  }
  bool STRUCTURE_ONLY = false;
  if (argc>4){
    STRUCTURE_ONLY = atoi(argv[4]) != 0;
  }

  bool DENSE = false;
  if (argc>5){
    DENSE = atoi(argv[5]) != 0;
  }

  cout << "PIXEL_NOISE: " <<  PIXEL_NOISE << endl;
  cout << "OUTLIER_RATIO: " << OUTLIER_RATIO<<  endl;
  cout << "ROBUST_KERNEL: " << ROBUST_KERNEL << endl;
  cout << "STRUCTURE_ONLY: " << STRUCTURE_ONLY<< endl;
  cout << "DENSE: "<<  DENSE << endl;

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(true);
  string solverName = "lm_fix6_3";
  if (DENSE) {
    solverName = "lm_dense6_3";
  } else {
#ifdef G2O_HAVE_CHOLMOD
    solverName = "lm_fix6_3_cholmod";
#else
    solverName = "lm_fix6_3";
#endif
  }

  g2o::OptimizationAlgorithmProperty solverProperty;
  optimizer.setAlgorithm(
      g2o::OptimizationAlgorithmFactory::instance()->construct(solverName, solverProperty));

  // add the marker corners
  vector<Vector3d> mk_corners;
  double mL = 21.74 * 0.001;  // big solpen
//   double mL = 0.01302; // small solpen
  // for (size_t i=0;i<500; ++i)
  // {
  mk_corners.push_back(Vector3d(-mL/2., mL/2., 0.));
  mk_corners.push_back(Vector3d( mL/2., mL/2., 0.));
  mk_corners.push_back(Vector3d( mL/2.,-mL/2., 0.));
  mk_corners.push_back(Vector3d(-mL/2.,-mL/2., 0.));
  // }

  // TODO
  double focal_length= 2.64474533e+03;
  Vector2d principal_point(1.00223569e+03, 5.77405442e+02);

  vector<g2o::SE3Quat,
      aligned_allocator<g2o::SE3Quat> > true_poses;
  g2o::CameraParameters * cam_params
      = new g2o::CameraParameters (focal_length, principal_point, 0.);
  cam_params->setId(0);

  if (!optimizer.addParameter(cam_params)) {
    assert(false);
  }

  // read all init poses (32+N_frame)-by-6
  vector<g2o::SE3Quat> cam_T_balls;
  read_rvec_tvec_txtfile("geom_txt_dat/cam_T_balls.txt", cam_T_balls);

  vector<g2o::SE3Quat> cent_T_faces;
  read_rvec_tvec_txtfile("geom_txt_dat/T_vec_cent_faces.txt", cent_T_faces);
  int num_markers = cent_T_faces.size();

  vector<vector<Vector2d>> aru_im_corners;
  read_aruco_corner_measurements("geom_txt_dat", aru_im_corners);

  vector<vector<int>> aru_ids;
  read_aruco_ids("geom_txt_dat", aru_ids);

    // adding vertices
    int vertex_id = 0;
    // int all_pose_cnt = cam_T_balls.size() + cent_T_faces.size();
    for (size_t i=0; i<cent_T_faces.size(); ++i) {
        
        g2o::SE3Quat pose = cent_T_faces[i];
        g2o::VertexSE3Expmap * v_se3
            = new g2o::VertexSE3Expmap();
        v_se3->setId(vertex_id);
        // if (i<2){
        //     v_se3->setFixed(true);
        // }
        v_se3->setEstimate(pose); 
        optimizer.addVertex(v_se3);
        // true_poses.push_back(pose);

        vertex_id++;
    }

    std::cout << "done adding cent_T_faces. Total vertices = " << vertex_id << std::endl;  

    for (size_t i=0; i<cam_T_balls.size(); ++i) {
        
        g2o::SE3Quat pose = cam_T_balls[i];
        g2o::VertexSE3Expmap * v_se3
            = new g2o::VertexSE3Expmap();
        v_se3->setId(vertex_id);
        // if (i<2){
        //     v_se3->setFixed(true);
        // }
        v_se3->setEstimate(pose);  
        optimizer.addVertex(v_se3);
        // true_poses.push_back(pose);

        vertex_id++;
    }

    std::cout << "done adding cam_T_balls. Total vertices = " << vertex_id << std::endl;  

    // create the graph
    int num_frames = cam_T_balls.size();
    vector<Vector2d> aru_im_p2d;
    vector<int> ids;
    for (size_t n_frm=0; n_frm<num_frames; ++n_frm) {
        ids = aru_ids[n_frm];
        g2o::SE3Quat cam_T_pen = cam_T_balls[n_frm];
        aru_im_p2d = aru_im_corners[n_frm];

        for (size_t n_mkid=0; n_mkid<ids.size(); n_mkid++) {
            int id = ids[n_mkid];
            g2o::SE3Quat pen_T_face = cent_T_faces[id];

            for (size_t i=0; i<mk_corners.size(); i++) {
                Vector3d aru_obj_pts = mk_corners[i];

                // add the edge
                g2o::EdgeProjectARU2UV * e = new g2o::EdgeProjectARU2UV();

                // ** measurement
                Vector2d z = aru_im_p2d[i];

                e->setMeasurement(z);
                e->information() = Matrix2d::Identity();
                if (ROBUST_KERNEL) {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                }

                // ** estimate
                g2o::VertexPointXYZ* vt_aru_obj_pts = new g2o::VertexPointXYZ();
                vt_aru_obj_pts->setEstimate(aru_obj_pts);
                vt_aru_obj_pts->setFixed(true);
                vt_aru_obj_pts->setId(vertex_id);
                ++vertex_id;
                optimizer.addVertex(vt_aru_obj_pts);
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vt_aru_obj_pts));

                e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertices().find(n_frm + num_markers)->second));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertices().find(n_frm)->second));
                e->setParameterId(0,0);
                optimizer.addEdge(e);

                // std::cout << "cam f " << e->_cam->focal_length << std::endl;
                // std::cout << "cam p " << e->_cam->principle_point << std::endl;

                // std::cout << "** check edge:" << std::endl;
                // std::cout << "e vtx 0: " << static_cast<g2o::VertexPointXYZ*>(e->vertices()[0])->estimate().transpose() << std::endl;
                // std::cout << "e vtx 1: " << static_cast<g2o::VertexSE3Expmap*>(e->vertices()[1])->estimate().translation().transpose() << std::endl;
                // std::cout << "e vtx 2: " << static_cast<g2o::VertexSE3Expmap*>(e->vertices()[2])->estimate().translation().transpose() << std::endl;

                // std::cout << std::endl;
                // std::cout << "** check optimizer:" << std::endl;
                g2o::SparseOptimizer::EdgeContainer es = optimizer.activeEdges();
                // for (int ii=0; ii<es.size(); ii++) {
                //     std::cout << "e :" << static_cast<g2o::VertexPointXYZ*>(es[ii]->vertices()[0])->estimate().transpose() << std::endl;
                //     std::cout << "e :" << static_cast<g2o::VertexSE3Expmap*>(es[ii]->vertices()[1])->estimate().translation().transpose() << std::endl;
                //     std::cout << "e :" << static_cast<g2o::VertexSE3Expmap*>(es[ii]->vertices()[2])->estimate().translation().transpose() << std::endl;
                // }                
                
                std::cout << "done adding an edge at obj pt " << vertex_id  << ", frame id: " << n_frm << std::endl;  
            }
        }
    }

    std::cout << "Finished creating the graph " << std::endl;  

    cout << endl;
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.setExportEveryIteration(true);
    std::cout << "Finished initialize optimization... " << std::endl;  

    if (STRUCTURE_ONLY){
        g2o::StructureOnlySolver<3> structure_only_ba;
        cout << "Performing structure-only BA:"   << endl;
        g2o::OptimizableGraph::VertexContainer points;
        for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
            g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
            if (v->dimension() == 3) {
                points.push_back(v);
            } else
            {
                std::cout << "vertex dim != 3" << std::endl;
            }
        }
        structure_only_ba.calc(points, 10);
    }
    optimizer.save("test.g2o");
    cout << endl;
    cout << "Performing full BA:" << endl;
    optimizer.optimize(2000);
    cout << endl;

    return 0;
}
