#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#ifndef PINOCCHIO_WITH_HPP_FCL
#define PINOCCHIO_WITH_HPP_FCL
#endif

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/collision/collision.hpp"
#include "pinocchio/collision/distance.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <Eigen/Core>
#include <boost/variant.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/home/prodefi/pinocchioDep/"
#endif

extern pinocchio::Model model;
extern pinocchio::Data data;
extern pinocchio::GeometryModel geom_model;
extern pinocchio::GeometryData geom_data;
extern Eigen::VectorXd q_init_;

void addObstacle(pinocchio::GeometryModel &geom_model);
void setupAndSimulateRobot();
bool CollisionCheck(const pinocchio::Model &model, pinocchio::Data &data,
                    const pinocchio::GeometryModel &geom_model,
                    pinocchio::GeometryData &geom_data,
                    const Eigen::VectorXd &q_total, bool verbose);
void printGeometryIDs(const pinocchio::GeometryModel &geom_model);
void printGeometrySizes(const pinocchio::GeometryModel &geom_model);
#endif // ROBOT_MODEL_H
