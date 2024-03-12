#pragma once

#include <fstream>
#include <mutex>

#include <basalt/utils/ba_utils.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <basalt/linearization/landmark_block.hpp>
#include <basalt/utils/sophus_utils.hpp>

namespace basalt {

template <typename Scalar, int POSE_SIZE>
class LandmarkBlockAbsDynamic : public LandmarkBlock<Scalar> {
 public:
  using Options = typename LandmarkBlock<Scalar>::Options;
  using State = typename LandmarkBlock<Scalar>::State;

  inline bool isNumericalFailure() const override {
    return state == State::NumericalFailure;
  }

  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;

  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  using Mat36 = Eigen::Matrix<Scalar, 3, 6>;

  using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
  using RowMatX =
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  virtual inline void allocateLandmark(
      Keypoint<Scalar>& lm,
      const Eigen::aligned_unordered_map<std::pair<TimeCamId, TimeCamId>,
                                         RelPoseLin<Scalar>>& relative_pose_lin,
      const Calibration<Scalar>& calib, const AbsOrderMap& aom, // aom的作用是为了判断观测点是否在abs_order_map中以及确定storage矩阵的列数
      const Options& options,
      const std::map<TimeCamId, size_t>* rel_order = nullptr) override {
    // some of the logic assumes the members are at their initial values
    BASALT_ASSERT(state == State::Uninitialized);

    UNUSED(rel_order);

    lm_ptr = &lm;
    options_ = &options;
    calib_ = &calib;

    // TODO: consider for VIO that we have a lot of 0 columns if we just use aom
    // --> add option to use different AOM with reduced size and/or just
    // involved poses --> when accumulating results, check which case we have;
    // if both aom are identical, we don't have to do block-wise operations.
    aom_ = &aom;

    pose_lin_vec.clear();
    pose_lin_vec.reserve(lm.obs.size());
    pose_tcid_vec.clear();
    pose_tcid_vec.reserve(lm.obs.size());

    // LMBs without host frame should not be created
    BASALT_ASSERT(aom.abs_order_map.count(lm.host_kf_id.frame_id) > 0);
    // std::cout << "host kf id=" << lm.host_kf_id.frame_id << std::endl;

    for (const auto& [tcid_t, pos] : lm.obs) {
      size_t i = pose_lin_vec.size();
      // std::cout << "target kf id=" << tcid_t.frame_id << std::endl;

      auto it = relative_pose_lin.find(std::make_pair(lm.host_kf_id, tcid_t));
      BASALT_ASSERT(it != relative_pose_lin.end());

      if (aom.abs_order_map.count(tcid_t.frame_id) > 0) { // 判断路标点的观测帧是否在abs_order_map中
        pose_lin_vec.push_back(&it->second); // 这里添加的是relative_pose_lin里面保存的RelPoseLin对象指针
      } else {
        // Observation droped for marginalization
        pose_lin_vec.push_back(nullptr);
      }
      pose_tcid_vec.push_back(&it->first);

      res_idx_by_abs_pose_[it->first.first.frame_id].insert(i);   // host
      res_idx_by_abs_pose_[it->first.second.frame_id].insert(i);  // target
    }

    // number of pose-jacobian columns is determined by oam
    padding_idx = aom_->total_size; // 位姿雅可比的列数由aom决定。

    // dense storage的行数由观测点的个数来决定，即：单个路标点的观测点的个数(pose_lin_vec.size()) * 残差维数(2) + 阻尼维数(3)
    num_rows = pose_lin_vec.size() * 2 + 3;  // residuals and lm damping 总行数的构成：残差2维 * 线性化位姿个数 + 最后3行的阻尼

    size_t pad = padding_idx % 4;
    if (pad != 0) {
      padding_size = 4 - pad; // 列数补齐为4的倍数，为了内存对齐
    }

    lm_idx = padding_idx + padding_size;
    res_idx = lm_idx + 3;
    num_cols = res_idx + 1; // 总列数的构成：若干列的位姿雅可比 + 3列的路标雅可比 + 1列的残差

    // number of columns should now be multiple of 4 for good memory alignment
    // TODO: test extending this to 8 --> 32byte alignment for float?
    BASALT_ASSERT(num_cols % 4 == 0);
    
    // 摘自rootba: We store only the blocks of the pose Jacobian that correspond to the poses where the landmark was observed
    // 存储能观测到该路标的pose Jacobian
    // 摘自rootba: also store the landmark’s Jacobians and residuals in the same landmark block
    // 相当于landmark block对应一个路标，它存储了路标的位姿雅可比，路标雅可比和残差。
    // 摘自rootba:Dense storage for the outlined (red) landmark block that efficiently stores all Jacobians and residuals for a single landmark.
    // 对于单个路标：[J_p | pad | J_l | res]: J_l是 2k_j*3的块，r是2k_j*1的块，J_p是2k_j*6的块。这里的k_j基本就是路标点所对应的target frame的个数
    storage.resize(num_rows, num_cols);
/*
 鉴于BA问题的特殊矩阵结构, 本文设计了一种高效内存占用的矩阵存储模式. 我们将BA的残差矩阵按照地图点来分组, 每个地图点j
 被 k_j 个相机观测到, 每个 J_l 子块的大小是2x3, J_p 子块的大小是2x6. 下图(b)则表示一个地图点相关的残差项landmark block, 
 可以看到我们把公式(6)的矩阵实现表示成一种很内存紧凑的形式, 它包括大小为 2k_j x 6 的位姿雅可比矩阵、大小为 2k_j x 3 的地图点雅可比矩阵
 和大小为 2k_j x 1 的重投影误差矩阵.

 Fig 2(c)表示使用in-place QR分解的边缘化操作后的landmark block, (c)下面一行残差块即为公式(17)的优化目标, (c)上面的残差块即为后带入项公式(16).

 Fig 3表示如何在一个已边缘化的landmark block上添加阻尼系数: 不是直接在大的雅可比矩阵 J_l 上添加对角阵 sqrt(λ)D_l , 我们在 J_l 下方添加一个小的3x3矩阵.

 这个额外的3x3绿色矩阵可以通过6 given rotations将其消掉:

 其中 Q_λ 是6 given rotation的乘积结果. 同理, 利用 Q_λ^T 也可以把阻尼项 sqrt(λ)D_l 给复原到原来位置.

 使用共轭梯度线性求解器.
 系统可以对每个地图点的landmark block独立的处理线性化、边缘化、两步求解工作, 所以可以直接并行化计算.
 
*/
/*
 * 2024-3-8补充说明：
 * 实际一个landmark的landmark block在位姿雅可比排列和布局上并不是像在rootba论文中Figure 2(b)那样的简单结构，
 * 在同一个二维行上面其实是可以有多个2x6的pose Jacobians, 即pose jacobians不会恰好是全部在对角线上
 * 
 * 2024-3-11补充说明:
 * 每一次optimize或者marg时,所有landmark block的storage矩阵的列数是一样的，而行数由每个landmark block的观测点个数决定
 */


    damping_rotations.clear();
    damping_rotations.reserve(6);

    state = State::Allocated;

    //std::cout << "lm_id:" << ;
    // std::cout << "obs.size:" << lm.obs.size() << " aom_->total_size:" << aom_->total_size << " num_rows:" << num_rows << " num_cols:" << num_cols << std::endl;
  }

  // may set state to NumericalFailure --> linearization at this state is
  // unusable. Numeric check is only performed for residuals that were
  // considered to be used (valid), which depends on
  // use_valid_projections_only setting.
  virtual inline Scalar linearizeLandmark() override {
    BASALT_ASSERT(state == State::Allocated ||
                  state == State::NumericalFailure ||
                  state == State::Linearized || state == State::Marginalized);

    // storage.setZero(num_rows, num_cols);
    storage.setZero(); // 存储矩阵设置为0矩阵
    damping_rotations.clear();
    damping_rotations.reserve(6); // 为了应用six Givens rotations.

    bool numerically_valid = true;

    Scalar error_sum = 0;
    std::cout << "observation number:" << lm_ptr->obs.size() << std::endl;
    size_t i = 0;
    for (const auto& [tcid_t, obs] : lm_ptr->obs) { // 遍历一个路标的所有观测
      std::visit(
          [&, obs = obs](const auto& cam) {
            // TODO: The pose_lin_vec[i] == nullptr is intended to deal with
            // dropped measurements during marginalization. However, dropped
            // measurements should only occur for the remaining frames, not for
            // the marginalized frames. Maybe these are observations bewtween
            // two marginalized frames, if more than one is marginalized at the
            // same time? But those we would not have to drop... Double check if
            // and when this happens and possibly resolve by fixing handling
            // here, or else updating the measurements in lmdb before calling
            // linearization. Otherwise, check where else we need a `if
            // (pose_lin_vec[i])` check or `pose_lin_vec[i] != nullptr` assert
            // in this class.

            if (pose_lin_vec[i]) { // 线性化相对位姿时，pose_lin_vec[i]所指向的RelPoseLin对象的成员已经被赋过值了。
              size_t obs_idx = i * 2; // 一个观测对应2维的重投影残差，所以乘以2
              // pose_tcid_vec[i]保存的是以host frame和target frame的id组成的pair
              // 而pose_lin_vec[i]保存的是RelPoseLin对象指针
              // 它们是从relative_pose_lin中查找指定landmark的观测得到的key和value，然后赋值得到的.
              size_t abs_h_idx =
                  aom_->abs_order_map.at(pose_tcid_vec[i]->first.frame_id)
                      .first; // 主导帧在storage矩阵中的列的序号
              size_t abs_t_idx =
                  aom_->abs_order_map.at(pose_tcid_vec[i]->second.frame_id)
                      .first; // 目标帧在storage矩阵中的列的序号

              std::cout << "abs_h_idx=" << abs_h_idx << " abs_t_idx=" << abs_t_idx << std::endl;
              Vec2 res;
              Eigen::Matrix<Scalar, 2, POSE_SIZE> d_res_d_xi;
              Eigen::Matrix<Scalar, 2, 3> d_res_d_p; // 残差对3d点的求导

              using CamT = std::decay_t<decltype(cam)>; // cam是具体的某一种相机模型对象
              // 使用相对姿态 T_t_h_sophus 来计算相对姿态的导数 d_res_d_xi， d_res_d_p
              bool valid = linearizePoint<Scalar, CamT>(
                  obs, *lm_ptr, pose_lin_vec[i]->T_t_h, cam, res, &d_res_d_xi,
                  &d_res_d_p); // obs是2x1的位置

              // use_valid_projections_only在landmakr_block.hpp中定义，默认为true.
              if (!options_->use_valid_projections_only || valid) { // 因此只有valid为true,才能进来
                numerically_valid = numerically_valid &&
                                    d_res_d_xi.array().isFinite().all() &&
                                    d_res_d_p.array().isFinite().all();

                const Scalar res_squared = res.squaredNorm(); // 2范数的平方
                const auto [weighted_error, weight] =
                    compute_error_weight(res_squared);
                const Scalar sqrt_weight =
                    std::sqrt(weight) / options_->obs_std_dev;

                // 统计一个路标的所有观测的加权重投影残差之和
                error_sum += weighted_error /
                             (options_->obs_std_dev * options_->obs_std_dev); // 加权的残差除以标准差（standard deviation）的平方

                // 索引obs_idx, lm_idx, abs_h_idx, abs_t_idx：
                // obs_idx是行序号: 0, 2, 4, ...
                // lm_idx是storage中路标jacobian的起始列序号
                // res_idx是storage中残差的起始列序号
                // abs_h_idx是主导帧在storage矩阵中的列的序号
                // abs_t_idx是目标帧在storage矩阵中的列的序号
                // abs_h_idx 和 abs_t_idx 均是aom.abs_order_map中每个6维的位姿帧按滑窗顺序进行排列的所在帧的维数序号
                storage.template block<2, 3>(obs_idx, lm_idx) =
                    sqrt_weight * d_res_d_p;
                storage.template block<2, 1>(obs_idx, res_idx) =
                    sqrt_weight * res;

                d_res_d_xi *= sqrt_weight;
                // 链式求导得到绝对位姿雅可比
                storage.template block<2, 6>(obs_idx, abs_h_idx) +=
                    d_res_d_xi * pose_lin_vec[i]->d_rel_d_h; // host frame的绝对位姿雅可比
                storage.template block<2, 6>(obs_idx, abs_t_idx) +=
                    d_res_d_xi * pose_lin_vec[i]->d_rel_d_t; // target frame的绝对位姿雅可比
              }
            }

            i++;
          },
          calib_->intrinsics[tcid_t.cam_id].variant); // variant是用泛型变量表示的相机模型对象
    }

    if (numerically_valid) {
      state = State::Linearized;
    } else {
      state = State::NumericalFailure;
    }

    return error_sum;
  }

  virtual inline void performQR() override {
    BASALT_ASSERT(state == State::Linearized);

    // 因为我们使用稠密矩阵，基于Householder变换的QR分解（“通过Householder变换实现QR分解”或者说“利用Householder变换进行QR分解”）可能更好
    // Since we use dense matrices Householder QR might be better:
    // https://mathoverflow.net/questions/227543/why-householder-reflection-is-better-than-givens-rotation-in-dense-linear-algebr

    if (options_->use_householder) { // use_householder默认为true.
      performQRHouseholder(); // 就地边缘化in-place marginalization
    } else {
      performQRGivens();
    }

    state = State::Marginalized;
  }

  // Sets damping and maintains upper triangular matrix for landmarks.
  virtual inline void setLandmarkDamping(Scalar lambda) override {
    BASALT_ASSERT(state == State::Marginalized);
    BASALT_ASSERT(lambda >= 0);

    if (hasLandmarkDamping()) {
      BASALT_ASSERT(damping_rotations.size() == 6);

      // undo dampening
      for (int n = 2; n >= 0; n--) {
        for (int m = n; m >= 0; m--) {
          storage.applyOnTheLeft(num_rows - 3 + n - m, n,
                                 damping_rotations.back().adjoint());
          damping_rotations.pop_back();
        }
      }
    }

    if (lambda == 0) {
      storage.template block<3, 3>(num_rows - 3, lm_idx).diagonal().setZero();
    } else {
      BASALT_ASSERT(Jl_col_scale.array().isFinite().all());

      storage.template block<3, 3>(num_rows - 3, lm_idx)
          .diagonal()
          .setConstant(sqrt(lambda));

      BASALT_ASSERT(damping_rotations.empty());

      // apply dampening and remember rotations to undo
      for (int n = 0; n < 3; n++) {
        for (int m = 0; m <= n; m++) {
          damping_rotations.emplace_back();
          damping_rotations.back().makeGivens(
              storage(n, lm_idx + n),
              storage(num_rows - 3 + n - m, lm_idx + n));
          storage.applyOnTheLeft(num_rows - 3 + n - m, n,
                                 damping_rotations.back());
        }
      }
    }
  }

  // lambda < 0 means computing exact model cost change
  // virtual inline void backSubstitute(const VecX& pose_inc,
  virtual inline bool backSubstitute(const VecX& pose_inc,
                                     Scalar& l_diff) override {
    BASALT_ASSERT(state == State::Marginalized);

    // For now we include all columns in LMB
    // padding_idx是位姿的总维数
    BASALT_ASSERT(pose_inc.size() == signed_cast(padding_idx));

    const auto Q1Jl = storage.template block<3, 3>(0, lm_idx)
                          .template triangularView<Eigen::Upper>(); // lwx: R_1

    const auto Q1Jr = storage.col(res_idx).template head<3>(); //lwx: Q_1^T r
    const auto Q1Jp = storage.topLeftCorner(3, padding_idx);//lwx: Q_1^T J_p

    // 根据Eigen 官方文档：
    // https://eigen.tuxfamily.org/dox/classEigen_1_1TriangularViewImpl_3_01MatrixType___00_01Mode___00_01Dense_01_4.html#ad0a79e600f86cad0d266d712fcec80b7
    // 示例m.triangularView<Eigen::Upper>().solve(n)等价于m.inverse()*n
    // 因此这儿路标增量等价于inc=-Q1Jl^{-1} * (Q1Jr + Q1Jp * pose_inc)
    // 即$inc=-R_1^{-1}(Q_1^T r + Q_1^T J_p {\Delta}x_p^*)$ 详见rootba (16).
    Vec3 inc = -Q1Jl.solve(Q1Jr + Q1Jp * pose_inc);

    // We want to compute the model cost change. The model function is
    //
    //     L(inc) = F(x) + inc^T J^T r + 0.5 inc^T J^T J inc
    //
    // and thus the expected decrease in cost for the computed increment is
    //
    //     l_diff = L(0) - L(inc)
    //            = - inc^T J^T r - 0.5 inc^T J^T J inc
    //            = - inc^T J^T (r + 0.5 J inc)
    //            = - (J inc)^T (r + 0.5 (J inc)).
    //
    // Here we have J = [Jp, Jl] under the orthogonal projection Q = [Q1, Q2],
    // i.e. the linearized system (model cost) is
    //00
    //    L(inc) = 0.5 || J inc + r ||^2 = 0.5 || Q^T J inc + Q^T r ||^2
    //
    // and below we thus compute
    //
    //    l_diff = - (Q^T J inc)^T (Q^T r + 0.5 (Q^T J inc)).
    //
    // We have
    //             | Q1^T |            | Q1^T Jp   Q1^T Jl |
    //    Q^T J =  |      | [Jp, Jl] = |                   |
    //             | Q2^T |            | Q2^T Jp      0    |.
    //
    // Note that Q2 is the nullspace of Jl, and Q1^T Jl == R. So with inc =
    // [incp^T, incl^T]^T we have
    //
    //                | Q1^T Jp incp + Q1^T Jl incl |
    //    Q^T J inc = |                             |
    //                | Q2^T Jp incp                |
    //

    // undo damping before we compute the model cost difference 在我们计算模型代价差之前取消阻尼
    setLandmarkDamping(0); // 搜了setLandmarkDamping，发现并未用到路标阻尼

    // compute "Q^T J incp" 计算 Q^T J_p {\Delta}x_p^*
    VecX QJinc = storage.topLeftCorner(num_rows - 3, padding_idx) * pose_inc;

    // add "Q1^T Jl incl" to the first 3 rows
    QJinc.template head<3>() += Q1Jl * inc; // 增加R_1 {\Delta}x_l^*

    auto Qr = storage.col(res_idx).head(num_rows - 3); //取出Q^T r
    // 计算公式：l_diff = - (J inc)^T (r + 0.5 (J inc)).
    // 计算公式：l_diff = - (Q^T J inc)^T (Q^T r + 0.5 (Q^T J inc)).
    l_diff -= QJinc.transpose() * (Scalar(0.5) * QJinc + Qr);

    // TODO: detect and handle case like ceres, allowing a few iterations but
    // stopping eventually
    if (!inc.array().isFinite().all() ||
        !lm_ptr->direction.array().isFinite().all() ||
        !std::isfinite(lm_ptr->inv_dist)) {
      std::cerr << "Numerical failure in backsubstitution\n";
      return false; // added by wxliu on 2023-12-19
    }

    // Note: scale only after computing model cost change
    inc.array() *= Jl_col_scale.array();

    // 更新路标增量：更新投影方向和逆深度
    lm_ptr->direction += inc.template head<2>();
    lm_ptr->inv_dist = std::max(Scalar(0), lm_ptr->inv_dist + inc[2]);

    return true; // added by wxliu on 2023-12-19
  }

  virtual inline size_t numReducedCams() const override {
    BASALT_LOG_FATAL("check what we mean by numReducedCams for absolute poses");
    return pose_lin_vec.size();
  }

  inline void addQ2JpTQ2Jp_mult_x(VecX& res,
                                  const VecX& x_pose) const override {
    UNUSED(res);
    UNUSED(x_pose);
    BASALT_LOG_FATAL("not implemented");
  }

  virtual inline void addQ2JpTQ2r(VecX& res) const override {
    UNUSED(res);
    BASALT_LOG_FATAL("not implemented");
  }

  virtual inline void addJp_diag2(VecX& res) const override {
    BASALT_ASSERT(state == State::Linearized);

    for (const auto& [frame_id, idx_set] : res_idx_by_abs_pose_) {
      const int pose_idx = aom_->abs_order_map.at(frame_id).first;
      for (const int i : idx_set) {
        const auto block = storage.block(2 * i, pose_idx, 2, POSE_SIZE);

        res.template segment<POSE_SIZE>(pose_idx) +=
            block.colwise().squaredNorm();
      }
    }
  }

  virtual inline void addQ2JpTQ2Jp_blockdiag(
      BlockDiagonalAccumulator<Scalar>& accu) const override {
    UNUSED(accu);
    BASALT_LOG_FATAL("not implemented");
  }

  virtual inline void scaleJl_cols() override {
    BASALT_ASSERT(state == State::Linearized);

    // ceres uses 1.0 / (1.0 + sqrt(SquaredColumnNorm))
    // we use 1.0 / (eps + sqrt(SquaredColumnNorm))
    Jl_col_scale =
        (options_->jacobi_scaling_eps +
         storage.block(0, lm_idx, num_rows - 3, 3).colwise().norm().array())
            .inverse();

    storage.block(0, lm_idx, num_rows - 3, 3) *= Jl_col_scale.asDiagonal();
  }

  virtual inline void scaleJp_cols(const VecX& jacobian_scaling) override {
    BASALT_ASSERT(state == State::Marginalized);

    // we assume we apply scaling before damping (we exclude the last 3 rows)
    BASALT_ASSERT(!hasLandmarkDamping());

    storage.topLeftCorner(num_rows - 3, padding_idx) *=
        jacobian_scaling.asDiagonal();
  }

  inline bool hasLandmarkDamping() const { return !damping_rotations.empty(); }

  virtual inline void printStorage(const std::string& filename) const override {
    std::ofstream f(filename);

    Eigen::IOFormat CleanFmt(4, 0, " ", "\n", "", "");

    f << "Storage (state: " << state
      << ", damping: " << (hasLandmarkDamping() ? "yes" : "no")
      << " Jl_col_scale: " << Jl_col_scale.transpose() << "):\n"
      << storage.format(CleanFmt) << std::endl;

    f.close();
  }
#if 0
  virtual inline void stage2(
      Scalar lambda, const VecX* jacobian_scaling, VecX* precond_diagonal2,
      BlockDiagonalAccumulator<Scalar>* precond_block_diagonal,
      VecX& bref) override {
    // 1. scale jacobian
    if (jacobian_scaling) {
      scaleJp_cols(*jacobian_scaling);
    }

    // 2. dampen landmarks
    setLandmarkDamping(lambda);

    // 3a. compute diagonal preconditioner (SCHUR_JACOBI_DIAGONAL)
    if (precond_diagonal2) {
      addQ2Jp_diag2(*precond_diagonal2);
    }

    // 3b. compute block diagonal preconditioner (SCHUR_JACOBI)
    if (precond_block_diagonal) {
      addQ2JpTQ2Jp_blockdiag(*precond_block_diagonal);
    }

    // 4. compute rhs of reduced camera normal equations
    addQ2JpTQ2r(bref);
  }
#endif

  inline State getState() const override { return state; }

  virtual inline size_t numQ2rows() const override { return num_rows - 3; }

 protected:
  inline void performQRGivens() {
    // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
    // See page 252, Algorithm 5.2.4 for how these two loops work
    Eigen::JacobiRotation<Scalar> gr;
    for (size_t n = 0; n < 3; n++) {
      for (size_t m = num_rows - 4; m > n; m--) {
        gr.makeGivens(storage(m - 1, lm_idx + n), storage(m, lm_idx + n)); // lm_idx是storage中路标jacobian的起始列序号
        storage.applyOnTheLeft(m, m - 1, gr);
      }
    }
  }

  inline void performQRHouseholder() {
    VecX tempVector1(num_cols);
    VecX tempVector2(num_rows - 3);

    for (size_t k = 0; k < 3; ++k) {
      size_t remainingRows = num_rows - k - 3;

      Scalar beta;
      Scalar tau;
      storage.col(lm_idx + k)
          .segment(k, remainingRows)
          .makeHouseholder(tempVector2, tau, beta);

      storage.block(k, 0, remainingRows, num_cols)
          .applyHouseholderOnTheLeft(tempVector2, tau, tempVector1.data());
    }
  }

  inline std::tuple<Scalar, Scalar> compute_error_weight(
      Scalar res_squared) const {
    // Note: Definition of cost is 0.5 ||r(x)||^2 to be in line with ceres

    if (options_->huber_parameter > 0) {
      // use huber norm
      const Scalar huber_weight =
          res_squared <= options_->huber_parameter * options_->huber_parameter
              ? Scalar(1)
              : options_->huber_parameter / std::sqrt(res_squared);
      const Scalar error =
          Scalar(0.5) * (2 - huber_weight) * huber_weight * res_squared;
      return {error, huber_weight};
    } else {
      // use squared norm
      return {Scalar(0.5) * res_squared, Scalar(1)};
    }
  }

  void get_dense_Q2Jp_Q2r(MatX& Q2Jp, VecX& Q2r,
                          size_t start_idx) const override {
    // 从storage中取出从res_idx（残差的起始列序号）列开始的末尾(num_rows - 3)行，
    // 也即storage最后一列的末尾(num_rows - 3)行，
    // 赋值到Q2r中从start_idx开始的(num_rows - 3)个元素上
    Q2r.segment(start_idx, num_rows - 3) =
        storage.col(res_idx).tail(num_rows - 3); //- 从storage中取出Q_2^T x r

    // padding_idx是位姿雅可比的总列数，即位姿的总维数。
    BASALT_ASSERT(Q2Jp.cols() == signed_cast(padding_idx));

    // 从storage中第3行第0列开始的(num_rows - 3)行和padding_idx列个元素，赋值到
    // Q2Jp的第start_idx开始，第0列的(num_rows - 3)行和padding_idx列个元素上。
    // 简单说：把storage中索引index(3, 0)开始的块block(num_rows - 3, padding_idx)上的元素赋值到
    // Q2Jp中索引(start_idx, 0)开始的块(num_rows - 3, padding_idx)上。
    Q2Jp.block(start_idx, 0, num_rows - 3, padding_idx) =
        storage.block(3, 0, num_rows - 3, padding_idx); //- 从storage中取出Q_2^T x J_p

  }

  void get_dense_Q2Jp_Q2r_rel(
      MatX& Q2Jp, VecX& Q2r, size_t start_idx,
      const std::map<TimeCamId, size_t>& rel_order) const override {
    UNUSED(Q2Jp);
    UNUSED(Q2r);
    UNUSED(start_idx);
    UNUSED(rel_order);
    BASALT_LOG_FATAL("not implemented");
  }

  void add_dense_H_b(DenseAccumulator<Scalar>& accum) const override {
    UNUSED(accum);
    BASALT_LOG_FATAL("not implemented");
  }

  void add_dense_H_b(MatX& H, VecX& b) const override {
    // 从storage矩阵取出landmark block的Q_2^T x r
    const auto r = storage.col(res_idx).tail(num_rows - 3);
    // 从storage矩阵取出landmark block的Q_2^T x J_p
    const auto J = storage.block(3, 0, num_rows - 3, padding_idx);

    // 则H的上波浪等于\widetilde{H} = J^T x J = (Q_2^T x J_p)^T x Q_2^T x J_p = J_p^T x Q_2 x Q_2^T x J_p
    H.noalias() += J.transpose() * J;
    // b的上波浪等于\widetilde{b} = J^T x r = (Q_2^T x J_p)^T x r = J_p^T x Q_2 x Q_2^T x r
    b.noalias() += J.transpose() * r;
  }

  void add_dense_H_b_rel(
      MatX& H_rel, VecX& b_rel,
      const std::map<TimeCamId, size_t>& rel_order) const override {
    UNUSED(H_rel);
    UNUSED(b_rel);
    UNUSED(rel_order);
    BASALT_LOG_FATAL("not implemented");
  }

  const Eigen::PermutationMatrix<Eigen::Dynamic>& get_rel_permutation()
      const override {
    BASALT_LOG_FATAL("not implemented");
  }

  Eigen::PermutationMatrix<Eigen::Dynamic> compute_rel_permutation(
      const std::map<TimeCamId, size_t>& rel_order) const override {
    UNUSED(rel_order);
    BASALT_LOG_FATAL("not implemented");
  }

  void add_dense_H_b_rel_2(MatX& H_rel, VecX& b_rel) const override {
    UNUSED(H_rel);
    UNUSED(b_rel);
    BASALT_LOG_FATAL("not implemented");
  }

  virtual TimeCamId getHostKf() const override { return lm_ptr->host_kf_id; }

 private:
  // Dense storage for pose Jacobians, padding, landmark Jacobians and
  // residuals [J_p | pad | J_l | res]
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      storage;

  Vec3 Jl_col_scale = Vec3::Ones();
  std::vector<Eigen::JacobiRotation<Scalar>> damping_rotations;

  std::vector<const RelPoseLin<Scalar>*> pose_lin_vec;
  std::vector<const std::pair<TimeCamId, TimeCamId>*> pose_tcid_vec;
  size_t padding_idx = 0;
  size_t padding_size = 0;
  size_t lm_idx = 0;
  size_t res_idx = 0;

  size_t num_cols = 0;
  size_t num_rows = 0;

  const Options* options_ = nullptr;

  State state = State::Uninitialized;

  Keypoint<Scalar>* lm_ptr = nullptr;
  const Calibration<Scalar>* calib_ = nullptr;
  const AbsOrderMap* aom_ = nullptr;

  std::map<int64_t, std::set<int>> res_idx_by_abs_pose_;
};

}  // namespace basalt
