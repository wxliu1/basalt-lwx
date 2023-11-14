
// created [wxliu 2023-6-10]
#include "imu_process.h"
#include "../wx_system.h"

extern bool USE_IMU;

namespace wx {
struct ImuProcess imu_; // wxliu 2023-6-16.
}

using namespace wx;

ImuProcess::ImuProcess()
{
    //if (sys_cfg_.use_imu) 
    {
        for (int i = 0; i < WINDOW_SIZE + 1; i++) {
            pre_integrations[i] = nullptr;
        }

        tmp_pre_integration = nullptr;

        for (int i = 0; i < NUM_OF_CAM; i++) { // move here. should run only once. 2023-7-19.
            tic[i] = Vector3d::Zero();
            ric[i] = Matrix3d::Identity();
        }

        clearState();

    }
    
}

#if 1
void ImuProcess::setExtrinsic(const Matrix3d& ric, const Vector3d& tic)
{
    this->ric[0] = ric;
    this->tic[0] = tic;

    std::cout << "[ImuProcess::setExtrinsic] : \n"
              << "ric= \n"
              <<  this->ric[0] << std::endl
              << "tic="
              <<  this->tic[0].transpose() << std::endl;
}

#else
void ImuProcess::setNodeHandle(const ros::NodeHandle& pnh)
{
    pnh_ = pnh;
    readExtrinsic();
    InitRosIO();
}

void ImuProcess::readExtrinsic()
{
    std::cout << "ImuProcess::readExtrinsic()\n";
    std::vector<double> vector_T{16, 0.0};
    pnh_.param<std::vector<double>>("body_T_cam0/data", vector_T, std::vector<double>());

    Eigen::Map<Eigen::Matrix4d> T(vector_T.data());
    Eigen::Matrix4d T2 = T.transpose();

    std::vector<Eigen::Matrix3d> RIC;
    std::vector<Eigen::Vector3d> TIC;
    RIC.push_back(T2.block<3, 3>(0, 0));
    TIC.push_back(T2.block<3, 1>(0, 3));
    tic[0] = TIC[0];
    ric[0] = RIC[0];

    std::cout << "body_T_cam0: \n" << T2 << std::endl;
}

void ImuProcess::InitRosIO()
{
    // tcpNoDelay是低延迟的tcp。
    sub_imu_ = pnh_.subscribe(wx::sys_cfg_.imu_topic, 2000, &ImuProcess::imu_callback, this, ros::TransportHints().tcpNoDelay());
    //sub_imu_ = pnh_.subscribe(wx::sys_cfg_.imu_topic, 200, &ImuProcess::imu_callback, this);
}
#endif

void ImuProcess::reset()
{
    initFirstPoseFlag = false;
    if (wx::sys_cfg_.use_imu)
    {
        clearState();
    }
}

void ImuProcess::clearState() {

  std::cout << " ImuProcess::clearState\n";
  
  imu_init_data_cnt = 0;
  accVector_.clear();

  mProcess.lock();
  while (!accBuf.empty()) accBuf.pop();
  while (!gyrBuf.empty()) gyrBuf.pop();
  // while(!featureBuf.empty())
  // featureBuf.pop();

  prevTime = -1;
  curTime = 0;

  initFirstPoseFlag = false;
  first_imu = false;

  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    Rs[i].setIdentity();
    Ps[i].setZero();
    Vs[i].setZero();
    Bas[i].setZero();
    Bgs[i].setZero();
    dt_buf[i].clear();
    linear_acceleration_buf[i].clear();
    angular_velocity_buf[i].clear();

    if (pre_integrations[i] != nullptr) {
      delete pre_integrations[i];
    }
    pre_integrations[i] = nullptr;
  }
/*
 * no use here. 2023-7-19
  for (int i = 0; i < NUM_OF_CAM; i++) {
    tic[i] = Vector3d::Zero();
    ric[i] = Matrix3d::Identity();
  }*/

  if (tmp_pre_integration != nullptr) delete tmp_pre_integration;

  tmp_pre_integration = nullptr;
  frame_count = 0;
  all_image_frame.clear();
  solver_flag = INITIAL;
  failure_occur = 0;

#if 0
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    sum_of_back = 0;
    sum_of_front = 0;   
    initial_timestamp = 0;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

#endif
  mProcess.unlock();
}

bool ImuProcess::ProcessData(double current_time) noexcept // wxliu 2023-6-12
{
    curTime = current_time;
    vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
    mBuf.lock();
    receive_imu = getIMUInterval(prevTime, curTime, accVector, gyrVector);
    mBuf.unlock();

    if (receive_imu) 
    {
      //std::cout << "initFirstPoseFlag " << initFirstPoseFlag << std::endl;
      if (!initFirstPoseFlag) {
#if 1
        // 2023-7-18
        imu_init_data_cnt++;

        /*if(imu_init_data_cnt <= 15 * 3)
        {
            return false;
        }
        else*/
        if(imu_init_data_cnt <= 15 * 1) // * n seconds
        {
            Eigen::Vector3d averAcc(0, 0, 0);
            int n = (int)accVector.size();
            for(size_t i = 0; i < accVector.size(); i++)
            {
                accVector_.emplace_back(accVector[i]);
            }

            if(imu_init_data_cnt == 15 * 1)
            {
                //std::cout << " accVector_.size=" << accVector_.size() << std::endl;
                initFirstIMUPose(accVector_);
            }
            else
            {
                return false;
            }

            
        }  
        // the end.
#else
        initFirstIMUPose(accVector); // comment 2023-7-18.
#endif
        // imu to world(imuToWolrd). 2023-4-3
        //const Sophus::SE3d tf(Rs[0], Vector3d(0, 0, 0)); // comment 2023-7-19

        //  tf为imu到world系的位姿， 乘以qbc后得到camera到world的位姿R_{w,c0}
        // camToWorld:  very important transfomation.
        Eigen::Matrix3d R_WC0 = Rs[0] * ric[0];
        //Vector3d t_WC0 = Vector3d(0, 0, 0) + Rs[0] * tic[0];
        Vector3d t_WC0 = Vector3d(0, 0, 0); // ensure the first CamToWorld pose's translation is '(0, 0, 0)' 2023-7-24

        // make sure the yaw of the first CamToWorld pose's rotation is 0. 2023-9-4.
        double yaw = Utility::R2ypr(R_WC0).x();
        R_WC0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R_WC0;
        // the end.


        //const Sophus::SE3d TWC0(R_WC0, t_WC0);  // comment 2023-7-11

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_WC0, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix3d U = svd.matrixU();
      Eigen::Matrix3d V = svd.matrixV();
      Eigen::Matrix3d R_ = U * (V.transpose());
      if (R_.determinant() < 0) {
        R_ = -R_;
      }
        
        // 2023-7-11
        //Sophus::SE3d TWC0(R_WC0, t_WC0);
        Sophus::SE3d TWC0(R_, t_WC0);

        Eigen::Matrix3d tmp_R;
        tmp_R << 0, 1, 0, -1, 0, 0, 0, 0, 1; // 绕z轴顺时针旋转-90度，使镜头对着 world frame 的 x 轴
        Vector3d tmp_t = Vector3d(0, 0, 0);
        Sophus::SE3d tmp_T(tmp_R, tmp_t);

        TWC0 = tmp_T * TWC0;
        // the end.

        // const Sophus::SE3d TWC0(Rs[0], Vector3d(0, 0, 0)); // modified  
        SetTwc0_(TWC0);
      }

      //double mean_dx = 0.0, mean_dy = 0.0, mean_dz = 0.0; // 2023-4-27.
      double mean_rx = 0.0, mean_ry = 0.0, mean_rz = 0.0; // 2023-5-15.

      for (size_t i = 0; i < accVector.size(); i++) {
        double dt;
        if (i == 0) {
          /* // comment 2023-4-3.
          if (prevTime < 0)
            prevTime = accVector[0].first;

          */

          dt = accVector[i].first - prevTime;
        } else if (i == accVector.size() - 1)
          dt = curTime - accVector[i - 1].first;
        else
          dt = accVector[i].first - accVector[i - 1].first;
        // LOG(INFO) << "[lwx] dt = " << dt << std::endl;
        processIMU(
            accVector[i].first, dt, accVector[i].second, gyrVector[i].second);

#ifdef _USE_IMU_ACC_
        mean_rx += fabs(gyrVector[i].second.x());
        mean_ry += fabs(gyrVector[i].second.y());
        mean_rz += fabs(gyrVector[i].second.z());
#endif
        //std::cout << "gyr:" << gyrVector[i].second.transpose() << std::endl;
      }

      prevTime = curTime;
#ifdef _USE_IMU_ACC_
      int gyr_size = gyrVector.size();
      mean_rx /= gyr_size;
      mean_ry /= gyr_size;
      mean_rz /= gyr_size;

      //if(mean_rx > 0.3 || mean_rz > 0.3)
      if(mean_rx > 0.5 || mean_rz > 0.5)
      //if(mean_rx > 0.3 || mean_rz > 0.5)
      {
        /*
        if(mean_rx > 0.1) std::cout << "PITCH PITCH  ";
        else if(mean_rz > 0.1) std::cout << "ROLL ROLL  ";*/
        
        /*std::cout << "mean_rx=" << mean_rx << "  mean_rz=" << mean_rz;
        std::cout << " The device is NOT horizontal\n";
        */
        is_device_horizontal = false;
        not_horizontal_cnt++;
      }
      else
      {
        //std::cout << " The device is horizontal\n";
        is_device_horizontal = true;
        not_horizontal_cnt = 0;
      }
#endif      
     

    }
    else 
    {
      return false;
    }
    

    return true;
}

void ImuProcess::UpdateImuPose(const Sophus::SE3d& T_w_cl) noexcept
{
    if(!use_imu_pose)
    {
        const Sophus::SE3d& tf = T_w_cl;
    
        Vector3d PCam = tf.translation();
        Matrix3d RCam = tf.rotationMatrix();

        // Rs[frame_count] = R;
        // Ps[frame_count] = P;

        // 这里的RCam是Rco_ck, Rs是Rco_bk, Ps是Pco_bk
        // 这里的c0指代world系
        Rs[frame_count] = RCam * ric[0].transpose();
        Ps[frame_count] = -RCam * ric[0].transpose() * tic[0] + PCam;
    }

}

void ImuProcess::NonlinearOptimization(double current_time)
{
    ImageFrame imageframe;  //(image, curr_header);
    imageframe.pre_integration = tmp_pre_integration;
    // all_image_frame.insert(make_pair(curr_header.stamp.toSec(), imageframe));
    //all_image_frame.emplace_back(curr_header.stamp.toSec(), imageframe);
    all_image_frame.emplace_back(current_time, imageframe);
    tmp_pre_integration =
        new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    // if(receive_imu && 1)// (solver_flag == INITIAL)
    if (receive_imu && solver_flag == INITIAL)  // (solver_flag == INITIAL)
    {
      // f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
      // f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
      if (frame_count == WINDOW_SIZE) {
        // map<double, ImageFrame>::iterator frame_it;
        vector<pair<double, ImageFrame>>::iterator frame_it;

        int i = 0;
        for (frame_it = all_image_frame.begin();
             frame_it != all_image_frame.end();
             frame_it++) {
          frame_it->second.R = Rs[i];
          frame_it->second.T = Ps[i];
          i++;
        }
        solveGyroscopeBias(all_image_frame, Bgs);
        for (int i = 0; i <= WINDOW_SIZE; i++) {
          // LOG(INFO) << "[lwx] i=" << i << "  Bgs[i]=" << Bgs[i].transpose()
          // << " pre_integrations[i]=" << pre_integrations[i] << std::endl;
          pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
          // pre_integrations[i]->repropagate(Vector3d::Zero(),
          // Vector3d(-0.0013659, -0.000218357, 0.000811628)); // for test.
          // 2023-3-29.
        }

        optimization();  // 2023-4-6
        solver_flag = NON_LINEAR;

        //is_not_horizontal_reboot = false; // 2023-5-16.
        SetNotHorizontalReboot_(false);

        /*
        optimization();
        updateLatestStates();
        solver_flag = NON_LINEAR;
        slideWindow();
        ROS_INFO("Initialization finish!");
        */
      } else if (frame_count < WINDOW_SIZE) {
        frame_count++;

        int prev_frame = frame_count - 1;
        Ps[frame_count] = Ps[prev_frame];
        Vs[frame_count] = Vs[prev_frame];
        Rs[frame_count] = Rs[prev_frame];
        Bas[frame_count] = Bas[prev_frame];
        Bgs[frame_count] = Bgs[prev_frame];
        
      }

    }

    else if (solver_flag == NON_LINEAR) {
      //std::cout << "before optimization: " << " Ps[frame_count]:" << Ps[frame_count].transpose() << " Bas[frame_count]:" << Bas[frame_count].transpose() << std::endl;
      optimization();
      //std::cout << "after optimization: " << " Ps[frame_count]:" << Ps[frame_count].transpose() << " Bas[frame_count]:" << Bas[frame_count].transpose() << std::endl;
    }
}

void ImuProcess::CalcImuPose(Sophus::SE3d& T_w_cl, bool camToWorld)
{
    if (camToWorld) {
      Eigen::Matrix3d R_WCk = Rs[frame_count] * ric[0];
      Vector3d t_WCk = Ps[frame_count] + Rs[frame_count] * tic[0];
      // svd
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(
          R_WCk, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix3d U = svd.matrixU();
      Eigen::Matrix3d V = svd.matrixV();
      Eigen::Matrix3d R_ = U * (V.transpose());
      if (R_.determinant() < 0) {
        R_ = -R_;
      }

      //t_WCk.x() += 1; // for test: visual tracked ok [wxliu 2023-6-9]
      //t_WCk.x() += 10; // for test: visual tracked failed [wxliu 2023-6-9]

      // const Sophus::SE3d tf(R_WCk, t_WCk);
      const Sophus::SE3d tf(R_, t_WCk);
      // std::cout << "[lwx] use imuToWorld transfer to camToWorld pose: tf= \n"
      //           << tf.matrix() << std::endl
      //           << std::endl;

      //PublishOdom2(header, tf); // comment 2023-5-15.

      T_w_cl = tf;

      //std::cout << "use_imu_pose=" << use_imu_pose << std::endl;
      // 2023-5-15.
 /*     tf_new = tf;
      Vector3d P = tf.translation();
      if(sys_cfg_.adjust_z_drift)
      {     
        double zdiff = fabs(P.z() - last_z);
        //if(zdiff > 0.01 && is_device_horizontal)
        if(zdiff > 0.001 && is_device_horizontal) // it's difficult to determine when should 'is_device_horizontal' is true or not.
        {
          std::cout << "zdiff = " << zdiff << " z-axis drift.\n";
          Vector3d p_new;
          p_new.x() = P.x();
          p_new.y() = P.y();
          p_new.z() = last_z;
          
          tf_new = Sophus::SE3d(tf.rotationMatrix(), p_new);
        }
        else// if(!is_device_horizontal)
        {
          // add 2023-5-15.

        if(not_horizontal_cnt >= 2)
        {
          last_z = P.z();
        }
        else
        {
          Vector3d p_new;
          p_new.x() = P.x();
          p_new.y() = P.y();
          p_new.z() = last_z;
          tf_new = Sophus::SE3d(tf.rotationMatrix(), p_new);
          odom_.frame.SetTwc(tf_new);
        }        
        // the end.
        }


      }

      
      
      PublishOdom2(header, tf_new); // 2023-5-15.
      // the end.

      //odom_.frame.SetTwc(tf);
      odom_.frame.SetTwc(tf_new); // 2023-5-15.
      odom_.frame.SetNewP(tf_new.translation()); // 2023-5-16.
*/
    } 
    else 
    {
      // svd
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(
          Rs[frame_count], Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix3d U = svd.matrixU();
      Eigen::Matrix3d V = svd.matrixV();
      Eigen::Matrix3d R_ = U * (V.transpose());
      if (R_.determinant() < 0) {
        R_ = -R_;
      }

      const Sophus::SE3d tf(R_, Ps[frame_count]);
      //PublishOdom2(header, tf); // comment 2023-6-12.

      Eigen::Matrix3d R_WCk = Rs[frame_count] * ric[0];
      Vector3d t_WCk = Ps[frame_count] + Rs[frame_count] * tic[0];

      Eigen::JacobiSVD<Eigen::Matrix3d> svd1(
          R_WCk, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix3d U1 = svd.matrixU();
      Eigen::Matrix3d V1 = svd.matrixV();
      Eigen::Matrix3d R1_ = U1 * (V1.transpose());
      if (R1_.determinant() < 0) {
        R1_ = -R1_;
      }

      T_w_cl = Sophus::SE3d(R1_, t_WCk);
      //odom_.frame.SetTwc(Sophus::SE3d(R1_, t_WCk));
    }
}

void ImuProcess::solveGyroscopeBias(vector<pair<double, ImageFrame>> &all_image_frame, Vector3d* Bgs)
{
    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero();
    b.setZero();
    //map<double, ImageFrame>::iterator frame_i;
    //map<double, ImageFrame>::iterator frame_j;

    vector<pair<double, ImageFrame>>::iterator frame_i;
    vector<pair<double, ImageFrame>>::iterator frame_j;

    //LOG(INFO) << "[lwx] solveGyroscopeBias all_image_frame.size()=" << all_image_frame.size() << std::endl;

    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);

        //LOG(INFO) << "[lwx] tmp_A=" << std::endl << tmp_A << std::endl;

        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    delta_bg = A.ldlt().solve(b);
    std::cout << "gyroscope bias initial calibration " << delta_bg.transpose();

    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] += delta_bg;

    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
    }
}

bool ImuProcess::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
                                
  if (accBuf.empty()) {
    printf("not receive imu\n");

    return false;
  }
  //printf("get imu from %f %f\n", t0, t1);
  //printf("imu front time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
  if (t1 <= accBuf.back().first) {
    while (accBuf.front().first <= t0) {
      accBuf.pop();
      gyrBuf.pop();
    }
    while (accBuf.front().first < t1) {
      accVector.push_back(accBuf.front());
      accBuf.pop();
      gyrVector.push_back(gyrBuf.front());
      gyrBuf.pop();
    }
    accVector.push_back(accBuf.front());
    gyrVector.push_back(gyrBuf.front());
  } else {
    printf("wait for imu\n");

    return false;
  }

  return true;
}

void ImuProcess::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
  if(!wx::sys_cfg_.use_imu)
  {
    return ;
  }

    mBuf.lock();
    accBuf.push(std::make_pair(t, linearAcceleration));
    gyrBuf.push(std::make_pair(t, angularVelocity));
    //printf("input imu with time %f \n", t);
    mBuf.unlock();
/*
    if (solver_flag == NON_LINEAR)
    {
        mPropagate.lock();
        fastPredictIMU(t, linearAcceleration, angularVelocity);
        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
        mPropagate.unlock();
    }
*/    
}

void ImuProcess::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    std::cout << "init R0 " << std::endl << Rs[0] << std::endl;
    //Vs[0] = Vector3d(5, 0, 0);
}

void ImuProcess::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

        //LOG(INFO) << "[lwx] frame_count=" << frame_count << "  pre_integrations[frame_count]=" << pre_integrations[frame_count] << std::endl;
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - sys_cfg_.g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - sys_cfg_.g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;

        
        #if 0
        // print delta translation after integration 2023-3-31
        Vector3d delta_t(0.0, 0.0, 0.0);
        delta_t = dt * Vs[j] + 0.5 * dt * dt * un_acc;
        std::cout << "print delta translation after integration: \n" << "delta_t=" << delta_t.transpose() << std::endl \
                  << "Ps[j]=" << Ps[j].transpose() << std::endl << std::endl;
        // the end.
        #endif

        Vs[j] += dt * un_acc;
/*
        // test 2023-8-25
        double theta_radian1 = atan(fabs(un_acc.z()) / (sqrt(un_acc.x() * un_acc.x() + un_acc.y() * un_acc.y())));
        std::cout << "theta_degree=" << (theta_radian1 / M_PI * 180) << " theta_radian=" << theta_radian1 << "  acc: " << un_acc.transpose() << std::endl;

        double theta_radian = atan(fabs(Vs[j].z()) / (sqrt(Vs[j].x() * Vs[j].x() + Vs[j].y() * Vs[j].y())));
        std::cout << "\033[33m" << "theta_degree=" << (theta_radian / M_PI * 180) << " theta_radian=" << theta_radian << "  velocity: " << Vs[j].transpose() << "\033[0m" << std::endl;
        // the end.
        */
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity; 
}

#if 0 // added by wxliu 2023-6-16
void ImuProcess::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    // 2023-3-31
    #if 0
    char szTime[60] = { 0 };
    sprintf(szTime, "%f", t);
    //LOG(INFO) << "[lwx] imu timestamp t=" << t << std::endl;
    std::cout << "[lwx] imu timestamp t=" << szTime << std::endl;
    #endif
    // the end.

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
/*
    // 2023-4-6
    dz = -imu_msg->linear_acceleration.y;
    dy = imu_msg->linear_acceleration.z;

    rz = -imu_msg->angular_velocity.y;
    ry = imu_msg->angular_velocity.z;
    // the end.
  */  

    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    //estimator.inputIMU(t, acc, gyr);
    inputIMU(t, acc, gyr); // modified 2023-3-23.

    //std::cout << "acc = " << acc.transpose() << std::endl;

    
    

 #if 0 
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();
   
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        // 只有初始化完成后才发送当前结果
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
#endif
}
#endif

void ImuProcess::slideWindow(int marg_frame_index)
{
  //
  if(solver_flag == NON_LINEAR)
  {
    if (frame_count == WINDOW_SIZE) 
    {
        // LOG(INFO) << "[lwx] frame_count=" << frame_count << "
        // marg_frame_index=" << odom_.marg_frame_index << std::endl;

        if (marg_frame_index == -1) {
        // marg old
            marg_frame_index = 0;
        }

        if (marg_frame_index >= 0) {
            auto [time, image_frame] = all_image_frame.at(marg_frame_index);
            delete image_frame.pre_integration;
            // image_frame.pre_integration = nullptr;
            all_image_frame.erase(all_image_frame.begin() + marg_frame_index);

            for (int i = marg_frame_index; i < WINDOW_SIZE; i++) {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                // Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }

            // Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{
                acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();
        }
    }

  }
}

void ImuProcess::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if(1)//(USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }
/*
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }


    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
*/    
}

void ImuProcess::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];
/*
    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }*/

    if(1)//(USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                          para_Pose[0][3],
                                                          para_Pose[0][4],
                                                          para_Pose[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        //TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            //ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5]).toRotationMatrix().transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;


                Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                            para_SpeedBias[i][1],
                                            para_SpeedBias[i][2]);

                Bas[i] = Vector3d(para_SpeedBias[i][3],
                                  para_SpeedBias[i][4],
                                  para_SpeedBias[i][5]);

                Bgs[i] = Vector3d(para_SpeedBias[i][6],
                                  para_SpeedBias[i][7],
                                  para_SpeedBias[i][8]);
            
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }
/*
    if(USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5]).normalized().toRotationMatrix();
        }
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if(USE_IMU)
        td = para_Td[0][0];
*/

}

void ImuProcess::optimization()
{
  
    //TicToc t_whole, t_prepare; // 2023-4-4.
    vector2double();

    ceres::Problem problem;
    
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
   
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if(USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if(!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);
/*
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            //ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            //ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);
*/
/*
 * tmp comment 2023-4-4
    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    */

    if(USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }
/*
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if(STEREO && it_per_frame.is_stereo)
            {                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
               
            }
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    //printf("prepare for ceres: %f \n", t_prepare.toc());
*/
    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;

    /*
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    */

   options.max_num_iterations = 8;
   options.max_solver_time_in_seconds = 0.04;

    //TicToc t_solver; // comment 2023-4-4
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    //ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    //printf("solver costs: %f \n", t_solver.toc());

    double2vector();
    //printf("frame_count: %d \n", frame_count);

    if(frame_count < WINDOW_SIZE)
        return;
    /*
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if(USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if(STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }*/
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //printf("whole time for ceres: %f \n", t_whole.toc());
    
}
