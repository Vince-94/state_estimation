#include "kalman_filter/extended_kalman_filter.hpp"

#include "utils/system_models.hpp"
#include "utils/jacobians.hpp"
#include "utils/custom_test_macros.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <iomanip>


// Fixture
class FixtureEKF : public ::testing::Test {
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
};


TEST_F(FixtureEKF, Constructor) {
    NonlinearSystem model(f, h, x0, u);
    ExtendedKalmanFilter ekf(std::move(model));

    SUCCEED() << "Constructor completed without crash";

    EXPECT_EQ(ekf.getStatePost().size(), 0);
    EXPECT_EQ(ekf.getCovariancePost().rows(), 0);
}


TEST_F(FixtureEKF, Initialize) {
    NonlinearSystem model(f, h, x0, u);
    ExtendedKalmanFilter ekf(std::move(model));

    ekf.initialize(x0, P0);

    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(nx);
    Eigen::MatrixXd zero_mat = Eigen::MatrixXd::Zero(nx, nx);

    EXPECT_MATRIX_NEAR(ekf.getStatePrio(), zero_vec, 1e-10, 6);
    EXPECT_MATRIX_NEAR(ekf.getCovariancePrio(), zero_mat, 1e-10, 6);

    EXPECT_MATRIX_NEAR(ekf.getStatePost(), x0, 1e-10, 6);
    EXPECT_MATRIX_NEAR(ekf.getCovariancePost(), P0, 1e-10, 6);
}


TEST_F(FixtureEKF, InitializeInvalidDimensions) {
    NonlinearSystem model(f, h, x0, u);
    ExtendedKalmanFilter ekf(std::move(model));

    Eigen::VectorXd bad_x(nx+1);
    EXPECT_THROW(ekf.initialize(bad_x, P0), std::invalid_argument);

    Eigen::MatrixXd bad_P(nx, nx+1);
    EXPECT_THROW(ekf.initialize(x0, bad_P), std::invalid_argument);
}


TEST_F(FixtureEKF, InitializeAtFirstUpdate) {
    NonlinearSystem model(f, h, x0, u);
    ExtendedKalmanFilter ekf(std::move(model));

    ekf.initializeAtFirstUpdate(z, R);

    Eigen::VectorXd expected_x_prio(nx);
    expected_x_prio << z(0), 0.0;

    Eigen::MatrixXd expected_P_prio(nx, nx);
    expected_P_prio << R(0,0), 0.0,
                       0.0,    1e9;

    EXPECT_MATRIX_NEAR(ekf.getStatePrio(), expected_x_prio, 1e-5, 6);
    EXPECT_MATRIX_NEAR(ekf.getCovariancePrio(), expected_P_prio, 1e5, 6);  // large_var tolerance

    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(nx);
    Eigen::MatrixXd zero_mat = Eigen::MatrixXd::Zero(nx, nx);

    EXPECT_MATRIX_NEAR(ekf.getStatePost(), zero_vec, 1e-10, 6);
    EXPECT_MATRIX_NEAR(ekf.getCovariancePost(), zero_mat, 1e-10, 6);
}


TEST_F(FixtureEKF, PredictInitialized) {
    NonlinearSystem model(f, h, x0, u);
    ExtendedKalmanFilter ekf(std::move(model));
    ekf.initialize(x0, P0);

    ekf.predict(u, Q);

    // Expected: pos = 0 + 5*0.1 + 0.01*25 = 0.5 + 0.25 = 0.75
    // vel = 5 + 0 = 5
    Eigen::VectorXd expected_x_prio(nx);
    expected_x_prio << 0.75, 5.0;

    // Approximate Jacobian df/dx at x=[0,5], u=0
    // ∂pos/∂pos = 1, ∂pos/∂vel = dt + 0.02*vel = 0.1 + 0.1 = 0.2
    // ∂vel/∂pos = 0, ∂vel/∂vel = 1
    Eigen::MatrixXd expected_Jfx(nx, nx);
    expected_Jfx << 1.0, 0.2,
                    0.0, 1.0;

    Eigen::MatrixXd expected_P_prio = expected_Jfx * P0 * expected_Jfx.transpose() + Q;

    EXPECT_MATRIX_NEAR(ekf.getStatePrio(), expected_x_prio, 1e-6, 6);
    EXPECT_MATRIX_NEAR(ekf.getCovariancePrio(), expected_P_prio, 1e-6, 6);
}


TEST_F(FixtureEKF, UpdateInitialized) {
    NonlinearSystem model(f, h, x0, u);
    ExtendedKalmanFilter ekf(std::move(model));
    ekf.initialize(x0, P0);

    ekf.predict(u, Q);
    ekf.update(z, R);

    // After predict + update with numerical Jacobian:
    // x_prio ≈ [0.75, 5.0]
    // y = 1 - 0.75 = 0.25
    // Jhx ≈ [1 + 0.02*5, 0] = [1.1, 0]
    // S ≈ 1.1 * P_prio(0,0) * 1.1 + 9 ≈ 12.1 + 9 ≈ 21.1 (approx)
    // K ≈ small values → x_post changes slightly from prior
    // But exact values depend on numerical Jacobian → check that update happened

    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(nx);
    Eigen::MatrixXd zero_mat = Eigen::MatrixXd::Zero(nx, nx);

    EXPECT_FALSE(ekf.getStatePost().isApprox(zero_vec, 1e-6))
        << "State should have been updated after measurement";

    EXPECT_FALSE(ekf.getCovariancePost().isApprox(zero_mat, 1e-6))
        << "Covariance should have been updated";

    EXPECT_TRUE(ekf.getCovariancePost().isApprox(ekf.getCovariancePost().transpose(), 1e-10))
        << "Covariance should be symmetric";
}


TEST_F(FixtureEKF, UpdateUninitialized) {
    NonlinearSystem model(f, h, x0, u);
    ExtendedKalmanFilter ekf(std::move(model));

    ekf.update(z, R);

    // EXPECT_TRUE(ekf.initialized_);  // TODO need a getter

    Eigen::VectorXd expected_x_prio(nx);
    expected_x_prio << z(0), 0.0;

    Eigen::MatrixXd expected_P_prio(nx, nx);
    expected_P_prio << R(0,0), 0.0,
                       0.0,    1e9;

    EXPECT_MATRIX_NEAR(ekf.getStatePrio(), expected_x_prio, 1e-5, 6);
    EXPECT_MATRIX_NEAR(ekf.getCovariancePrio(), expected_P_prio, 1e5, 6);

    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(nx);
    Eigen::MatrixXd zero_mat = Eigen::MatrixXd::Zero(nx, nx);

    EXPECT_MATRIX_NEAR(ekf.getStatePost(), zero_vec, 1e-10, 6);
    EXPECT_MATRIX_NEAR(ekf.getCovariancePost(), zero_mat, 1e-10, 6);
}


TEST_F(FixtureEKF, SettersAndGetters) {
    NonlinearSystem model(f, h, x0, u);
    ExtendedKalmanFilter ekf(std::move(model));

    ekf.setStatePrio(x0);
    EXPECT_TRUE(ekf.getStatePrio().isApprox(x0));

    ekf.setCovariancePrio(P0);
    EXPECT_TRUE(ekf.getCovariancePrio().isApprox(P0));

    ekf.setStatePost(x0);
    EXPECT_TRUE(ekf.getStatePost().isApprox(x0));

    ekf.setCovariancePost(P0);
    EXPECT_TRUE(ekf.getCovariancePost().isApprox(P0));
}
