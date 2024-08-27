#define PINOCCHIO_WITH_HPP_FCL

#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/collision/collision.hpp"
#include "pinocchio/collision/distance.hpp"
#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "boost/variant.hpp"

// PINOCCHIO_MODEL_DIR 由 CMake 定义，但您可以在此处定义自己的目录。
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/home/prodefi/pinocchioDep/"
#endif
using namespace pinocchio;

int main(int /*argc*/, char ** /*argv*/) {
  const std::string robots_model_path = PINOCCHIO_MODEL_DIR;

  // 设置 URDF 文件路径
  const std::string urdf_filename =
      robots_model_path + "model/limbarm_robot.xacro.urdf";
  // 设置 SRDF 文件路径
  const std::string srdf_filename =
      robots_model_path + "model/limbarm_robot.srdf";

  // 加载 URDF 模型
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);
  // 打印link的名称与索引
  for (size_t i = 0; i < model.njoints; ++i) {
    std::cout << "Link " << i << " : " << model.names[i] << std::endl;
  }
  // 创建与模型关联的数据
  Data data(model);

  // 加载 URDF 文件中的几何数据
  GeometryModel geom_model;
  pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION,
                             geom_model, robots_model_path);

  // 添加所有可能的碰撞对并移除 SRDF 文件中列出的碰撞对
  geom_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model, geom_model, srdf_filename);

  // 创建与几何模型关联的数据
  GeometryData geom_data(geom_model);

  // // 加载机器人的参考配置
  pinocchio::srdf::loadReferenceConfigurations(model, srdf_filename);
  // 分别获取每个臂部的初始配置
  Eigen::VectorXd q1 = model.referenceConfigurations.at("init_arm1");
  Eigen::VectorXd q2 = model.referenceConfigurations.at("init_arm2");
  Eigen::VectorXd q3 = model.referenceConfigurations.at("init_arm3");
  Eigen::VectorXd q4 = model.referenceConfigurations.at("init_arm4");

  // 假设我们只需要每个向量的部分数据来构成28维向量
  // 此处仅为示例，具体情况取决于实际模型配置
  Eigen::VectorXd q_total(28);
  q_total << q1.head(7), q2.segment(7, 7), q3.segment(14, 7), q4.tail(7);

  std::cout << "Combined q: " << q_total.transpose() << std::endl;
  // 进行碰撞检测
  pinocchio::computeCollisions(model, data, geom_model, geom_data, q_total);
  // 打印所有碰撞对的状态
  for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k) {
    const CollisionPair &cp = geom_model.collisionPairs[k];
    const hpp::fcl::CollisionResult &cr = geom_data.collisionResults[k];

    std::cout << "collision pair: " << cp.first << " , " << cp.second
              << " - collision: ";
    std::cout << (cr.isCollision() ? "yes" : "no") << std::endl;
  }

  return 0;
}
