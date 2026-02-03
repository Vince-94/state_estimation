#include "kalman_filter/unscented_kalman_filter.hpp"
#include "utils/system_models.hpp"
#include "utils/jacobians.hpp"
#include "utils/custom_test_macros.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <iomanip>


// Fixture
class FixtureUKF : public ::testing::Test {
protected:
    void SetUp() override {
        nx = 2;
        nu = 1;
        ny = 1;

        double dt = 0.1;

        // Nonlinear state transition: pos += vel*dt + 0.01*vel² + u*dt²/2
        f = [dt](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
            Eigen::VectorXd x_next(2);
            x_next(0) = x(0) + x(1)*dt + 0.01 * x(1)*x(1) + u(0)*dt*dt/2.0;
            x_next(1) = x(1) + u(0)*dt;
            return x_next;
        };

        // Measurement: observe position only
        h = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
            (void)u;  // suppress unused warning
            return (Eigen::VectorXd(1) << x(0)).finished();
        };

        x0.resize(nx);
        x0 << 0.0, 5.0;

        P0.resize(nx, nx);
        P0 = Eigen::MatrixXd::Identity(nx, nx) * 10.0;

        u.resize(nu);
        u.setZero();

        Q.resize(nx, nx);
        Q = Eigen::MatrixXd::Identity(nx, nx) * 0.01;

        R.resize(ny, ny);
        R = Eigen::MatrixXd::Identity(ny, ny) * 9.0;

        z.resize(ny);
        z << 1.0;

        alpha = 1e-3;
        kappa = 0.0;
        beta = 2.0;
    }

    int nx, nu, ny;

    NonlinearSystem::DynFunc f;
    NonlinearSystem::DynFunc h;

    Eigen::VectorXd x0;
    Eigen::MatrixXd P0;
    Eigen::VectorXd u;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::VectorXd z;

    double alpha;
    double kappa;
    double beta;
};


TEST_F(FixtureUKF, Constructor) {
    NonlinearSystem model(f, h, x0, u);
    UnscentedKalmanFilter ukf(std::move(model), alpha, kappa, beta);

    SUCCEED() << "Constructor completed without crash";

    EXPECT_EQ(ukf.getStatePost().size(), 0);
    EXPECT_EQ(ukf.getCovariancePost().rows(), 0);
}


TEST_F(FixtureUKF, Initialize) {
    NonlinearSystem model(f, h, x0, u);
    UnscentedKalmanFilter ukf(std::move(model), alpha, kappa, beta);

    ukf.initialize(x0, P0);

    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(nx);
    Eigen::MatrixXd zero_mat = Eigen::MatrixXd::Zero(nx, nx);

    EXPECT_MATRIX_NEAR(ukf.getStatePrio(), zero_vec, 1e-10, 6);
    EXPECT_MATRIX_NEAR(ukf.getCovariancePrio(), zero_mat, 1e-10, 6);

    EXPECT_MATRIX_NEAR(ukf.getStatePost(), x0, 1e-10, 6);
    EXPECT_MATRIX_NEAR(ukf.getCovariancePost(), P0, 1e-10, 6);
}


TEST_F(FixtureUKF, InitializeInvalidDimensions) {
    NonlinearSystem model(f, h, x0, u);
    UnscentedKalmanFilter ukf(std::move(model), alpha, kappa, beta);

    Eigen::VectorXd bad_x(nx+1);
    EXPECT_THROW(ukf.initialize(bad_x, P0), std::invalid_argument);

    Eigen::MatrixXd bad_P(nx, nx+1);
    EXPECT_THROW(ukf.initialize(x0, bad_P), std::invalid_argument);
}


TEST_F(FixtureUKF, InitializeAtFirstUpdate) {
    NonlinearSystem model(f, h, x0, u);
    UnscentedKalmanFilter ukf(std::move(model), alpha, kappa, beta);

    ukf.initializeAtFirstUpdate(z, R);

    Eigen::VectorXd expected_x_prio(nx);
    expected_x_prio << z(0), 0.0;

    Eigen::MatrixXd expected_P_prio(nx, nx);
    expected_P_prio << R(0,0), 0.0,
                       0.0,    1e9;

    EXPECT_MATRIX_NEAR(ukf.getStatePrio(), expected_x_prio, 1e-5, 6);
    EXPECT_MATRIX_NEAR(ukf.getCovariancePrio(), expected_P_prio, 1e5, 6);  // large_var tolerance

    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(nx);
    Eigen::MatrixXd zero_mat = Eigen::MatrixXd::Zero(nx, nx);

    EXPECT_MATRIX_NEAR(ukf.getStatePost(), zero_vec, 1e-10, 6);
    EXPECT_MATRIX_NEAR(ukf.getCovariancePost(), zero_mat, 1e-10, 6);
}


TEST_F(FixtureUKF, PredictInitialized) {
    NonlinearSystem model(f, h, x0, u);
    UnscentedKalmanFilter ukf(std::move(model), alpha, kappa, beta);
    ukf.initialize(x0, P0);

    ukf.predict(u, Q);

    Eigen::VectorXd x_prio = ukf.getStatePrio();

    // Position should be > 0.75 due to convex nonlinearity
    EXPECT_GT(x_prio(0), 0.75) << "Nonlinear effect should increase mean position";

    // Velocity should remain close to 5
    EXPECT_NEAR(x_prio(1), 5.0, 0.01);

    Eigen::MatrixXd P_prio = ukf.getCovariancePrio();

    // Diagonal should be larger than initial + Q due to nonlinearity
    EXPECT_GT(P_prio(0,0), 10.0 + 0.01);
    EXPECT_NEAR(P_prio(1,1), 10.01, 0.1);

    // Symmetry
    EXPECT_MATRIX_NEAR(P_prio, P_prio.transpose(), 1e-10, 6);
}


TEST_F(FixtureUKF, UpdateInitialized) {
    NonlinearSystem model(f, h, x0, u);
    UnscentedKalmanFilter ukf(std::move(model), alpha, kappa, beta);
    ukf.initialize(x0, P0);

    ukf.predict(u, Q);
    ukf.update(z, R);

    // After predict + update with sigma points
    // x_prio ≈ [0.75, 5.0]
    // z_pred ≈ 0.75
    // y = 1 - 0.75 = 0.25
    // S, P_xz from sigma propagation → update state/cov
    // Check update happened and cov symmetric

    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(nx);
    Eigen::MatrixXd zero_mat = Eigen::MatrixXd::Zero(nx, nx);

    EXPECT_FALSE(ukf.getStatePost().isApprox(zero_vec, 1e-6))
        << "State should have been updated after measurement";

    EXPECT_FALSE(ukf.getCovariancePost().isApprox(zero_mat, 1e-6))
        << "Covariance should have been updated";

    EXPECT_MATRIX_NEAR(ukf.getCovariancePost(), ukf.getCovariancePost().transpose(), 1e-10, 6)
        << "Covariance should be symmetric";
}


TEST_F(FixtureUKF, UpdateUninitialized) {
    NonlinearSystem model(f, h, x0, u);
    UnscentedKalmanFilter ukf(std::move(model), alpha, kappa, beta);

    ukf.update(z, R);

    // EXPECT_TRUE(ukf.initialized_);  // TODO need a getter

    Eigen::VectorXd expected_x_prio(nx);
    expected_x_prio << z(0), 0.0;

    Eigen::MatrixXd expected_P_prio(nx, nx);
    expected_P_prio << R(0,0), 0.0,
                       0.0,    1e9;

    EXPECT_MATRIX_NEAR(ukf.getStatePrio(), expected_x_prio, 1e-5, 6);
    EXPECT_MATRIX_NEAR(ukf.getCovariancePrio(), expected_P_prio, 1e5, 6);

    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(nx);
    Eigen::MatrixXd zero_mat = Eigen::MatrixXd::Zero(nx, nx);

    EXPECT_MATRIX_NEAR(ukf.getStatePost(), zero_vec, 1e-10, 6);
    EXPECT_MATRIX_NEAR(ukf.getCovariancePost(), zero_mat, 1e-10, 6);
}


TEST_F(FixtureUKF, SettersAndGetters) {
    NonlinearSystem model(f, h, x0, u);
    UnscentedKalmanFilter ukf(std::move(model), alpha, kappa, beta);

    ukf.setStatePrio(x0);
    EXPECT_TRUE(ukf.getStatePrio().isApprox(x0));

    ukf.setCovariancePrio(P0);
    EXPECT_TRUE(ukf.getCovariancePrio().isApprox(P0));

    ukf.setStatePost(x0);
    EXPECT_TRUE(ukf.getStatePost().isApprox(x0));

    ukf.setCovariancePost(P0);
    EXPECT_TRUE(ukf.getCovariancePost().isApprox(P0));
}
