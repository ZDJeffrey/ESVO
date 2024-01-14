#include <esvo_core/core/RegProblemSolverLM.h>
#include <esvo_core/tools/cayley.h>
<<<<<<< HEAD
#include <thread>
=======
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b

namespace esvo_core
{
namespace core
{
RegProblemSolverLM::RegProblemSolverLM(
  esvo_core::CameraSystem::Ptr &camSysPtr,
  shared_ptr<RegProblemConfig> &rpConfigPtr,
  esvo_core::core::RegProblemType rpType,
  size_t numThread):
  camSysPtr_(camSysPtr),
  rpConfigPtr_(rpConfigPtr),
  rpType_(rpType),
  NUM_THREAD_(numThread),
  bPrint_(false),
<<<<<<< HEAD
  bVisualize_(true),
  initialized_(false)
=======
  bVisualize_(true)
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
{
  if(rpType_ == REG_NUMERICAL)
  {
    numDiff_regProblemPtr_ =
      std::make_shared<Eigen::NumericalDiff<RegProblemLM> >(camSysPtr_, rpConfigPtr_, NUM_THREAD_);
  }
  else if(rpType_ == REG_ANALYTICAL)
  {
    regProblemPtr_ = std::make_shared<RegProblemLM>(camSysPtr_, rpConfigPtr_, NUM_THREAD_);
<<<<<<< HEAD
    regProblemPtr_right_ = std::make_shared<RegProblemLM>(camSysPtr_, rpConfigPtr_, NUM_THREAD_);
=======
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
  }
  else
  {
    LOG(ERROR) << "Wrong Registration Problem Type is assigned!!!";
    exit(-1);
  }
  z_min_ = 1.0 / rpConfigPtr_->invDepth_max_range_;
  z_max_ = 1.0 / rpConfigPtr_->invDepth_min_range_;

  lmStatics_.nPoints_ = 0;
  lmStatics_.nfev_ = 0;
  lmStatics_.nIter_ = 0;
<<<<<<< HEAD

  T_right_left_ = Eigen::Matrix4d::Identity();
  T_right_left_.block<3,4>(0,0) = camSysPtr_->T_right_left_;

  f_ = camSysPtr_->cam_left_ptr_->P_(0,0); // 焦距
  baseline_ = camSysPtr_->baseline_; // 基线
=======
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
}

RegProblemSolverLM::~RegProblemSolverLM()
{}

bool RegProblemSolverLM::resetRegProblem(RefFrame* ref, CurFrame* cur)
{
<<<<<<< HEAD
  if(!initialized_) // 未初始化
  {
    LOG(INFO)<<"Initialize ResItems.";
    // LOG(INFO)<<"\nref tr:\n"<<ref->tr_.getTransformationMatrix()<<"\ncur tr\n"<<cur->tr_.getTransformationMatrix()<<"\ncur tr_right\n"<<cur->tr_right_.getTransformationMatrix();
    initialized_ = true;
    // 获取旋转平移
    T_world_ref_ = ref->tr_.getTransformationMatrix();
    Eigen::Matrix3d R_world_ref = T_world_ref_.block<3,3>(0,0);
    Eigen::Vector3d t_world_ref = T_world_ref_.block<3,1>(0,3);
    // 设置残差项
    ResItems_.clear();
    size_t numPoints = ref->vPointXYZPtr_.size();
    ResItems_.resize(numPoints);
    for(size_t i = 0;i < numPoints;i++)
    {
      std::swap(ref->vPointXYZPtr_[i], ref->vPointXYZPtr_[i + rand() % (numPoints - i)]);
      Eigen::Vector3d p_tmp(ref->vPointXYZPtr_[i]->x, ref->vPointXYZPtr_[i]->y, ref->vPointXYZPtr_[i]->z); // 世界坐标系
      Eigen::Vector3d p_cam = R_world_ref.transpose() * (p_tmp - t_world_ref); // ref坐标系
      ResItems_[i].initialize(p_cam(0), p_cam(1), p_cam(2));
      camSysPtr_->cam_left_ptr_->world2Cam(ResItems_[i].p_, ResItems_[i].p_img_);
    }
    // 输出ref深度图
    if(bVisualize_)
    {
      size_t width = camSysPtr_->cam_left_ptr_->width_;
      size_t height = camSysPtr_->cam_left_ptr_->height_;
      cv::Mat depth_ref = cv::Mat(cv::Size(width,height),CV_32FC1,cv::Scalar(0));
      for(auto& ResItem:ResItems_) // 遍历所有点
        if(ResItem.p_img_(0)>=0 && ResItem.p_img_(0)<width && ResItem.p_img_(1)>=0 && ResItem.p_img_(1)<height)
          depth_ref.at<float>(ResItem.p_img_(1),ResItem.p_img_(0)) = ResItem.p_(2); // 设置深度值
      std_msgs::Header header;
      header.stamp = ref->t_;
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "32FC1", depth_ref).toImageMsg();
      depthMap_ref_pub_->publish(msg); // 话题输出
    }
  }

=======
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
  if(cur->numEventsSinceLastObs_ < rpConfigPtr_->MIN_NUM_EVENTS_)
  {
    LOG(INFO) << "resetRegProblem RESET fails for no enough events coming in.";
    LOG(INFO) << "However, the system remains to work.";
  }
<<<<<<< HEAD
  if( ResItems_.size() < rpConfigPtr_->BATCH_SIZE_ )
  {
    LOG(INFO) << "resetRegProblem RESET fails for no enough point cloud in the local map.";
    LOG(INFO) << "The system will be re-initialized";
    initialized_ = false;
=======
  if( ref->vPointXYZPtr_.size() < rpConfigPtr_->BATCH_SIZE_ )
  {
    LOG(INFO) << "resetRegProblem RESET fails for no enough point cloud in the local map.";
    LOG(INFO) << "The system will be re-initialized";
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
    return false;
  }
  //  LOG(INFO) << "resetRegProblem RESET succeeds.";
  if(rpType_ == REG_NUMERICAL)
  {
<<<<<<< HEAD
    numDiff_regProblemPtr_->setProblem(T_world_ref_, cur, &ResItems_, false);
=======
    numDiff_regProblemPtr_->setProblem(ref, cur, false);
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
//    LOG(INFO) << "numDiff_regProblemPtr_->setProblem(ref, cur, false) -----------------";
  }
  if(rpType_ == REG_ANALYTICAL)
  {
<<<<<<< HEAD
    regProblemPtr_->setProblem(T_world_ref_, cur, &ResItems_, true, true);
    regProblemPtr_right_->setProblem(T_world_ref_, cur, &ResItems_, true, false);
=======
    regProblemPtr_->setProblem(ref, cur, true);
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
//    LOG(INFO) << "regProblemPtr_->setProblem(ref, cur, true) -----------------";
  }

  lmStatics_.nPoints_ = 0;
  lmStatics_.nfev_ = 0;
  lmStatics_.nIter_ = 0;
  return true;
}

bool RegProblemSolverLM::solve_numerical()
{
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<RegProblemLM>, double> lm(*numDiff_regProblemPtr_.get());
  lm.resetParameters();
  lm.parameters.ftol = 1e-3;
  lm.parameters.xtol = 1e-3;
  lm.parameters.maxfev = rpConfigPtr_->MAX_ITERATION_ * 8;

  size_t iteration = 0;
  size_t nfev = 0;
  while(true)
  {
    if(iteration >= rpConfigPtr_->MAX_ITERATION_)
      break;
    numDiff_regProblemPtr_->setStochasticSampling(
      (iteration % numDiff_regProblemPtr_->numBatches_) * rpConfigPtr_->BATCH_SIZE_, rpConfigPtr_->BATCH_SIZE_);
    Eigen::VectorXd x(6);
    x.fill(0.0);
    if(lm.minimizeInit(x) == Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
    {
      LOG(ERROR) << "ImproperInputParameters for LM (Tracking)." << std::endl;
      return false;
    }

    Eigen::LevenbergMarquardtSpace::Status status = lm.minimizeOneStep(x);
    numDiff_regProblemPtr_->addMotionUpdate(x);

    iteration++;
    nfev += lm.nfev;

    /*************************** Visualization ************************/
    if(bVisualize_)// will slow down the tracker's performance a little bit
    {
      size_t width = camSysPtr_->cam_left_ptr_->width_;
      size_t height = camSysPtr_->cam_left_ptr_->height_;
      cv::Mat reprojMap_left = cv::Mat(cv::Size(width, height), CV_8UC1, cv::Scalar(0));
      cv::eigen2cv(numDiff_regProblemPtr_->cur_->pTsObs_->TS_negative_left_, reprojMap_left);
      reprojMap_left.convertTo(reprojMap_left, CV_8UC1);
      cv::cvtColor(reprojMap_left, reprojMap_left, CV_GRAY2BGR);

      // project 3D points to current frame
      Eigen::Matrix3d R_cur_ref =  numDiff_regProblemPtr_->R_.transpose();
      Eigen::Vector3d t_cur_ref = -numDiff_regProblemPtr_->R_.transpose() * numDiff_regProblemPtr_->t_;

<<<<<<< HEAD
      size_t numVisualization = std::min(numDiff_regProblemPtr_->pResItems_->size(), (size_t)2000);
      for(size_t i = 0; i < numVisualization; i++)
      {
        ResidualItem & ri = (*numDiff_regProblemPtr_->pResItems_)[i];
=======
      size_t numVisualization = std::min(numDiff_regProblemPtr_->ResItems_.size(), (size_t)2000);
      for(size_t i = 0; i < numVisualization; i++)
      {
        ResidualItem & ri = numDiff_regProblemPtr_->ResItems_[i];
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
        Eigen::Vector3d p_3D = R_cur_ref * ri.p_ + t_cur_ref;
        Eigen::Vector2d p_img_left;
        camSysPtr_->cam_left_ptr_->world2Cam(p_3D, p_img_left);
        double z = ri.p_[2];
        visualizor_.DrawPoint(1.0 / z, 1.0 / z_min_, 1.0 / z_max_,
                              Eigen::Vector2d(p_img_left(0), p_img_left(1)), reprojMap_left);
      }
      std_msgs::Header header;
      header.stamp = numDiff_regProblemPtr_->cur_->t_;
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", reprojMap_left).toImageMsg();
      reprojMap_pub_->publish(msg);
    }
    /*************************** Visualization ************************/
    if(status == 2 || status == 3)
      break;
  }
//  LOG(INFO) << "LM Finished ...................";
  numDiff_regProblemPtr_->setPose();
  lmStatics_.nPoints_ = numDiff_regProblemPtr_->numPoints_;
  lmStatics_.nfev_ = nfev;
  lmStatics_.nIter_ = iteration;
  return 0;
}

<<<<<<< HEAD
bool RegProblemSolverLM::solve(std::shared_ptr<RegProblemLM> regProblemPtr)
{
  Eigen::LevenbergMarquardt<RegProblemLM, double> lm(*regProblemPtr.get());
=======
bool RegProblemSolverLM::solve_analytical()
{
  Eigen::LevenbergMarquardt<RegProblemLM, double> lm(*regProblemPtr_.get());
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
  lm.resetParameters();
  lm.parameters.ftol = 1e-3;
  lm.parameters.xtol = 1e-3;
  lm.parameters.maxfev = rpConfigPtr_->MAX_ITERATION_ * 8;

  size_t iteration = 0;
  size_t nfev = 0;
  while(true)
  {
    if(iteration >= rpConfigPtr_->MAX_ITERATION_)
      break;
<<<<<<< HEAD
    regProblemPtr->setStochasticSampling(
      (iteration % regProblemPtr->numBatches_) * rpConfigPtr_->BATCH_SIZE_, rpConfigPtr_->BATCH_SIZE_);
=======
    regProblemPtr_->setStochasticSampling(
      (iteration % regProblemPtr_->numBatches_) * rpConfigPtr_->BATCH_SIZE_, rpConfigPtr_->BATCH_SIZE_);
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
    Eigen::VectorXd x(6);
    x.fill(0.0);
    if(lm.minimizeInit(x) == Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
    {
      LOG(ERROR) << "ImproperInputParameters for LM (Tracking)." << std::endl;
      return false;
    }
    Eigen::LevenbergMarquardtSpace::Status status = lm.minimizeOneStep(x);
<<<<<<< HEAD
    regProblemPtr->addMotionUpdate(x);
=======
    regProblemPtr_->addMotionUpdate(x);
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b

    iteration++;
    nfev += lm.nfev;
    if(status == 2 || status == 3)
      break;
  }
<<<<<<< HEAD
  regProblemPtr->setPose();
  return true;
}

bool RegProblemSolverLM::solve_analytical()
{
  std::thread thread_left(&RegProblemSolverLM::solve, this, regProblemPtr_);
  std::thread thread_right(&RegProblemSolverLM::solve, this, regProblemPtr_right_);
  if(thread_left.joinable())
    thread_left.join();
  if(thread_right.joinable())
    thread_right.join();

  Eigen::Matrix4d T_ref_left = Eigen::Matrix4d::Identity();
  T_ref_left.block<3,3>(0,0) = regProblemPtr_->R_;
  T_ref_left.block<3,1>(0,3) = regProblemPtr_->t_;
  Eigen::Matrix4d T_ref_right = Eigen::Matrix4d::Identity();
  T_ref_right.block<3,3>(0,0) = regProblemPtr_right_->R_;
  T_ref_right.block<3,1>(0,3) = regProblemPtr_right_->t_;
  Eigen::Matrix4d T_left_right = T_ref_left.inverse() * T_ref_right;
  Eigen::Matrix4d T_error = T_right_left_ * T_left_right;
  double translational_error = sqrt(pow(T_error(0,3),2) + pow(T_error(1,3),2) + pow(T_error(2,3),2));
  // LOG(INFO)<<"translational_error: "<<translational_error; // test

  double trans_threshold = 0.3;
  if(translational_error > trans_threshold)
  {
    LOG(INFO)<<"Slove failed as the translation error "<<translational_error<<" > "<<trans_threshold<<" Item size:"<<ResItems_.size();
    initialized_ = false;
    return false;
  }

  size_t width = camSysPtr_->cam_left_ptr_->width_;
  size_t height = camSysPtr_->cam_left_ptr_->height_;
  // project 3D points to current frame
  Eigen::Matrix3d R_left_ref =  regProblemPtr_->R_.transpose();
  Eigen::Vector3d t_left_ref = -regProblemPtr_->R_.transpose() * regProblemPtr_->t_;
  Eigen::Matrix3d R_right_ref =  regProblemPtr_right_->R_.transpose();
  Eigen::Vector3d t_right_ref = -regProblemPtr_right_->R_.transpose() * regProblemPtr_right_->t_;

  for(auto ResItem = ResItems_.begin(); ResItem!=ResItems_.end(); ResItem++) // 遍历ref所有点
  {
    Eigen::Vector3d p_ref = ResItem->p_;
    Eigen::Vector3d p_3D_left = R_left_ref * p_ref + t_left_ref;
    Eigen::Vector3d p_3D_right = R_right_ref * p_ref + t_right_ref;
    Eigen::Vector2d p_img_left, p_img_right; // 像素坐标
    camSysPtr_->cam_left_ptr_->world2Cam(p_3D_left, p_img_left);
    camSysPtr_->cam_right_ptr_->world2Cam(p_3D_right, p_img_right);

    double d = p_img_left(0) - p_img_right(0); // 视差

    if(p_img_left(0) < 0 || p_img_left(0) >= width || p_img_left(1) < 0 || p_img_left(1) >= height ||
       p_img_right(0) < 0 || p_img_right(0) >= width || p_img_right(1) < 0 || p_img_right(1) >= height ||
       d<=0) // 无法计算深度
    {
      ResItems_.erase(ResItem); //  删除该点
      ResItem--;
      continue;
    }
    double inv_depth = d / (f_ * baseline_); // 逆深度
    camSysPtr_->cam_left_ptr_->cam2World(p_img_left,inv_depth,ResItem->p_);
    ResItem->p_img_ = p_img_left;
  }

  T_world_ref_ = regProblemPtr_->getPose(); // 设置当前帧为下一次优化的参考帧
//   LOG(INFO)<<"ResItem size:"<<ResItems_.size(); // test
=======
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b

  /*************************** Visualization ************************/
  if(bVisualize_) // will slow down the tracker a little bit
  {
<<<<<<< HEAD
    cv::Mat depthMap_cur = cv::Mat(cv::Size(width, height), CV_32FC1, cv::Scalar(0)); // 深度图

    cv::Mat reprojMap_left = cv::Mat(cv::Size(width, height), CV_8UC1, cv::Scalar(0)); // 重映射图
=======
    size_t width = camSysPtr_->cam_left_ptr_->width_;
    size_t height = camSysPtr_->cam_left_ptr_->height_;
    cv::Mat reprojMap_left = cv::Mat(cv::Size(width, height), CV_8UC1, cv::Scalar(0));
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
    cv::eigen2cv(regProblemPtr_->cur_->pTsObs_->TS_negative_left_, reprojMap_left);
    reprojMap_left.convertTo(reprojMap_left, CV_8UC1);
    cv::cvtColor(reprojMap_left, reprojMap_left, CV_GRAY2BGR);

<<<<<<< HEAD
    for(auto& ResItem : ResItems_) // 遍历ref所有点
    {
      depthMap_cur.at<float>(ResItem.p_img_(1), ResItem.p_img_(0)) = ResItem.p_(2); // 设置深度
      visualizor_.DrawPoint(1.0 / ResItem.p_(2), 1.0 / z_min_, 1.0 / z_max_,
                            Eigen::Vector2d(ResItem.p_img_(0), ResItem.p_img_(1)), reprojMap_left);
    }
    std_msgs::Header header;
    header.stamp = regProblemPtr_->cur_->t_;
    
    sensor_msgs::ImagePtr depthMap_msg = cv_bridge::CvImage(header,"32FC1",depthMap_cur).toImageMsg();
    depthMap_cur_pub_->publish(depthMap_msg);

=======
    // project 3D points to current frame
    Eigen::Matrix3d R_cur_ref =  regProblemPtr_->R_.transpose();
    Eigen::Vector3d t_cur_ref = -regProblemPtr_->R_.transpose() * regProblemPtr_->t_;

    size_t numVisualization = std::min(regProblemPtr_->ResItems_.size(), (size_t)2000);
    for(size_t i = 0; i < numVisualization; i++)
    {
      ResidualItem & ri = regProblemPtr_->ResItems_[i];
      Eigen::Vector3d p_3D = R_cur_ref * ri.p_ + t_cur_ref;
      Eigen::Vector2d p_img_left;
      camSysPtr_->cam_left_ptr_->world2Cam(p_3D, p_img_left);
      double z = ri.p_[2];
      visualizor_.DrawPoint(1.0 / z, 1.0 / z_min_, 1.0 / z_max_,
                            Eigen::Vector2d(p_img_left(0), p_img_left(1)), reprojMap_left);
    }
    std_msgs::Header header;
    header.stamp = regProblemPtr_->cur_->t_;
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", reprojMap_left).toImageMsg();
    reprojMap_pub_->publish(msg);
  }
  /*************************** Visualization ************************/

<<<<<<< HEAD
  return true;
}

void RegProblemSolverLM::setRegPublisher(image_transport::Publisher* reprojMap_pub,
                                         image_transport::Publisher* depth_ref_pub,
                                         image_transport::Publisher* depth_cur_pub)
{
  reprojMap_pub_ = reprojMap_pub;
  depthMap_ref_pub_ = depth_ref_pub;
  depthMap_cur_pub_ = depth_cur_pub;
=======
  regProblemPtr_->setPose();
  lmStatics_.nPoints_ = regProblemPtr_->numPoints_;
  lmStatics_.nfev_ = nfev;
  lmStatics_.nIter_ = iteration;
  return 0;
}

void RegProblemSolverLM::setRegPublisher(
  image_transport::Publisher* reprojMap_pub)
{
  reprojMap_pub_ = reprojMap_pub;
>>>>>>> fb90dea0b24cf2cb8580ecfbc49355882b3f5c8b
}

}//namespace core
}//namespace esvo_core

