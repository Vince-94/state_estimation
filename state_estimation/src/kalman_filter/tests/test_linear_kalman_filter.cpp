#include "kalman_filter/linear_kalman_filter.hpp"
#include "utils/system_models.hpp"
#include "utils/custom_test_macros.hpp"

#include <Eigen/Dense>
#include <stdexcept>

#include <gtest/gtest.h>
#include <gmock/gmock.h>


// Fixture
class FixtureLKF : public ::testing::Test {
protected:
    void SetUp() override {
        nx = 2;
        nu = 1;
        ny = 1;

        double dt = 0.1;

        A.resize(nx, nx);
        A << 1.0, dt,
             0.0, 1.0;
        B.resize(nx, nu);
        B << dt*dt/2.0,
             dt;
        C.resize(ny, nx);
        C << 1.0, 0.0;

        x0.resize(nx);
        x0 << 0.0, 5.0;  // pos=0, vel=5

        P0.resize(nx, nx);
        P0 = Eigen::MatrixXd::Identity(nx, nx) * 10.0;  // uncertain

        u.resize(nu);
        u << 0.0;  // no accel

        Q.resize(nx, nx);
        Q = Eigen::MatrixXd::Identity(nx, nx) * 0.01;  // small process noise

        R.resize(ny, ny);
        R = Eigen::MatrixXd::Identity(ny, ny) * 9.0;   // gps var=9 (std=3)

        z.resize(ny);
        z << 1.0;  // sample measurement
    }

    int nx;
    int nu;
    int ny;

    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;

    Eigen::VectorXd x0;
    Eigen::MatrixXd P0;
    Eigen::VectorXd u;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::VectorXd z;
};


TEST_F(FixtureLKF, Initialize) {
    StateSpace model(A, B, C);
    LinearKalmanFilter lkf(std::move(model));

    lkf.initialize(x0, P0);

    EXPECT_MATRIX_NEAR(lkf.getStatePrio(), Eigen::VectorXd::Zero(nx), 1e-10, 6);
    EXPECT_MATRIX_NEAR(lkf.getCovariancePrio(), Eigen::MatrixXd::Zero(nx, nx), 1e-10, 6);

    EXPECT_MATRIX_NEAR(lkf.getStatePost(), x0, 1e-10, 6);
    EXPECT_MATRIX_NEAR(lkf.getCovariancePost(), P0, 1e-10, 6);
}


TEST_F(FixtureLKF, InitializeAlreadyInitialized) {
    StateSpace model(A, B, C);
    LinearKalmanFilter lkf(std::move(model));

    lkf.initialize(x0, P0);

    lkf.initialize(x0, P0);  // Should do nothing, check unchanged

    EXPECT_MATRIX_NEAR(lkf.getStatePost(), x0, 1e-10, 6);
}


TEST_F(FixtureLKF, InitializeInvalidDimensions) {
    StateSpace model(A, B, C);
    LinearKalmanFilter lkf(std::move(model));

    Eigen::VectorXd bad_x(nx+1);
    EXPECT_THROW(lkf.initialize(bad_x, P0), std::invalid_argument);

    Eigen::MatrixXd bad_P(nx, nx+1);
    EXPECT_THROW(lkf.initialize(x0, bad_P), std::invalid_argument);
}


// TODO fails since assumes a proper pseudoinverse -> fix in the code
TEST_F(FixtureLKF, InitializeAtFirstUpdate) {
    StateSpace model(A, B, C);
    LinearKalmanFilter lkf(std::move(model));

    lkf.initializeAtFirstUpdate(z, R);

    // For singular case, we expect:
    // x_prio ≈ [z(0), 0] (projected)
    // P_prio(0,0) ≈ R(0,0) = 9, others small/large depending on implementation
    Eigen::VectorXd expected_x_prio(nx);
    expected_x_prio << z(0), 0.0;

    EXPECT_MATRIX_NEAR(lkf.getStatePrio(), expected_x_prio, 1e-10, 6) << "x_prio: " << lkf.getStatePrio().transpose();

    // Covariance: at least check diagonal and scale
    EXPECT_NEAR(lkf.getCovariancePrio()(0,0), 9.0, 1e-3);  // var(pos) ≈ R
    EXPECT_NEAR(lkf.getCovariancePrio()(1,1), 0.0, 1e-3);  // or large if using pseudoinverse with big var

    EXPECT_MATRIX_NEAR(lkf.getStatePost(), Eigen::VectorXd::Zero(nx), 1e-10, 6);
    EXPECT_MATRIX_NEAR(lkf.getCovariancePost(), Eigen::MatrixXd::Zero(nx, nx), 1e-10, 6);
}


TEST_F(FixtureLKF, InitializeAtFirstUpdateInvalidDimensions) {
    StateSpace model(A, B, C);
    LinearKalmanFilter lkf(std::move(model));

    Eigen::VectorXd bad_z(ny+1);
    EXPECT_THROW(lkf.initializeAtFirstUpdate(bad_z, R), std::invalid_argument);

    Eigen::MatrixXd bad_R(ny+1, ny+1);
    EXPECT_THROW(lkf.initializeAtFirstUpdate(z, bad_R), std::invalid_argument);
}


TEST_F(FixtureLKF, PredictInitialized) {
    StateSpace model(A, B, C);
    LinearKalmanFilter lkf(std::move(model));
    lkf.initialize(x0, P0);

    lkf.predict(u, Q);

    Eigen::VectorXd expected_x_prio = A * x0 + B * u;
    EXPECT_MATRIX_NEAR(lkf.getStatePrio(), expected_x_prio, 1e-10, 6);

    Eigen::MatrixXd expected_P_prio = A * P0 * A.transpose() + Q;
    EXPECT_MATRIX_NEAR(lkf.getCovariancePrio(), expected_P_prio, 1e-10, 6);
}


TEST_F(FixtureLKF, PredictUninitialized) {
    StateSpace model(A, B, C);
    LinearKalmanFilter lkf(std::move(model));

    lkf.predict(u, Q);

    EXPECT_TRUE(lkf.getStatePrio().isZero());
    EXPECT_TRUE(lkf.getCovariancePrio().isZero());
}


TEST_F(FixtureLKF, PredictInvalidDimensions) {
    StateSpace model(A, B, C);
    LinearKalmanFilter lkf(std::move(model));
    lkf.initialize(x0, P0);

    Eigen::VectorXd bad_u(nu+1);
    EXPECT_THROW(lkf.predict(bad_u, Q), std::invalid_argument);

    Eigen::MatrixXd bad_Q(nx+1, nx+1);
    EXPECT_THROW(lkf.predict(u, bad_Q), std::invalid_argument);
}


TEST_F(FixtureLKF, UpdateInitialized) {
    StateSpace model(A, B, C);
    LinearKalmanFilter lkf(std::move(model));
    lkf.initialize(x0, P0);

    lkf.update(z, R);

    Eigen::VectorXd y = z - C * x0;
    Eigen::MatrixXd S = C * P0 * C.transpose() + R;
    Eigen::MatrixXd K = P0 * C.transpose() * S.inverse();
    Eigen::VectorXd expected_x_post = x0 + K * y;
    Eigen::MatrixXd expected_P_post = (Eigen::MatrixXd::Identity(nx, nx) - K * C) * P0;

    EXPECT_MATRIX_NEAR(lkf.getStatePost(), expected_x_post, 1e-10, 6);
    EXPECT_MATRIX_NEAR(lkf.getCovariancePost(), expected_P_post, 1e-10, 6);
    EXPECT_MATRIX_NEAR(lkf.getCovariancePost().transpose(), lkf.getCovariancePost(), 1e-10, 6);  // Symmetric
}


// TODO fails since assumes a proper pseudoinverse -> fix in the code
TEST_F(FixtureLKF, UpdateUninitialized) {
    StateSpace model(A, B, C);
    LinearKalmanFilter lkf(std::move(model));

    lkf.update(z, R);

    auto Ct_Rinv = C.transpose() * R.inverse();
    auto info = Ct_Rinv * C;
    auto expected_P_prio = info.inverse();
    auto expected_x_prio = expected_P_prio * (Ct_Rinv * z);

    EXPECT_MATRIX_NEAR(lkf.getStatePrio(), expected_x_prio, 1e-10, 6);
    EXPECT_MATRIX_NEAR(lkf.getCovariancePrio(), expected_P_prio, 1e-10, 6);

    EXPECT_MATRIX_NEAR(lkf.getStatePost(), Eigen::VectorXd::Zero(nx), 1e-10, 6);  // Symmetric
    EXPECT_MATRIX_NEAR(lkf.getCovariancePost(), Eigen::MatrixXd::Zero(nx, nx), 1e-10, 6);  // Symmetric
}


TEST_F(FixtureLKF, UpdateInvalidDimensions) {
    StateSpace model(A, B, C);
    LinearKalmanFilter lkf(std::move(model));
    lkf.initialize(x0, P0);

    Eigen::VectorXd bad_z(ny+1);
    EXPECT_THROW(lkf.update(bad_z, R), std::invalid_argument);

    Eigen::MatrixXd bad_R(ny+1, ny+1);
    EXPECT_THROW(lkf.update(z, bad_R), std::invalid_argument);
}


TEST_F(FixtureLKF, SettersAndGetters) {
    StateSpace model(A, B, C);
    LinearKalmanFilter lkf(std::move(model));

    lkf.setStatePrio(x0);
    EXPECT_MATRIX_NEAR(lkf.getStatePrio(), x0, 1e-10, 6);

    lkf.setCovariancePrio(P0);
    EXPECT_MATRIX_NEAR(lkf.getCovariancePrio(), P0, 1e-10, 6);

    lkf.setStatePost(x0);
    EXPECT_MATRIX_NEAR(lkf.getStatePost(), x0, 1e-10, 6);

    lkf.setCovariancePost(P0);
    EXPECT_MATRIX_NEAR(lkf.getCovariancePost(), P0, 1e-10, 6);
}
