#ifndef ESVO_CORE_CORE_REGPROBLEMSOLVERLM_H
#define ESVO_CORE_CORE_REGPROBLEMSOLVERLM_H

#include <memory>
#include <esvo_core/core/RegProblemLM.h>
#include <esvo_core/tools/Visualization.h>
#include <esvo_core/optimization/OptimizationFunctor.h>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

namespace esvo_core
{
namespace core
{
enum RegProblemType
{
  REG_NUMERICAL,
  REG_ANALYTICAL
};

struct LM_statics
{
  size_t nPoints_;
  size_t nfev_;
  size_t nIter_;
};

class RegProblemSolverLM
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RegProblemSolverLM(
    CameraSystem::Ptr& camSysPtr,
    std::shared_ptr<RegProblemConfig>& rpConfigPtr,
    RegProblemType rpType = REG_NUMERICAL,
    size_t numThread = 1);
  virtual ~RegProblemSolverLM();

  bool resetRegProblem(RefFrame* ref, CurFrame* cur);
  bool solve_numerical();// relatively slower
  bool solve_analytical();// faster
<<<<<<< HEAD
  bool solve(std::shared_ptr<RegProblemLM> job);

  // For test and visualization
  void setRegPublisher(image_transport::Publisher* reprojMap_pub,
                       image_transport::Publisher* depthMap_ref_pub,
                       image_transport::Publisher* depthMap_cur_pub);
  LM_statics lmStatics_;// record LevenburgMarquardt log.

  // ref信息
  ResidualItems ResItems_; // 左右位姿优化共享残差项
  bool initialized_; // 是否已初始化
  image_transport::Publisher *depthMap_ref_pub_, *depthMap_cur_pub_; // 深度图输出
  Eigen::Matrix4d T_right_left_, T_world_ref_;
  double f_; // 焦距
  double baseline_; // 基线

=======

  // For test and visualization
  void setRegPublisher(image_transport::Publisher* reprojMap_pub);
  LM_statics lmStatics_;// record LevenburgMarquardt log.

>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
  // variables
private:
  CameraSystem::Ptr& camSysPtr_;
  std::shared_ptr<RegProblemConfig> rpConfigPtr_;
  size_t NUM_THREAD_;
  RegProblemType rpType_;

<<<<<<< HEAD
  std::shared_ptr<RegProblemLM> regProblemPtr_,regProblemPtr_right_;
=======
  std::shared_ptr<RegProblemLM> regProblemPtr_;
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
  std::shared_ptr<Eigen::NumericalDiff<RegProblemLM> > numDiff_regProblemPtr_;

  // For test
  double z_min_, z_max_;
  image_transport::Publisher *reprojMap_pub_;
  Visualization visualizor_;
  bool bPrint_, bVisualize_;
};
}//namespace core
}//namespace esvo_core
#endif //ESVO_CORE_CORE_REGPROBLEMSOLVER2_H