package org.firstinspires.ftc.teamcode.ftc7083.filter;

/**
 * A Kalman Filter is an efficient algorithm used to estimate the state of a dynamic system from a
 * series of noisy measurements. It is widely used in various fields, including control systems,
 * robotics, computer vision, and navigation, to obtain accurate and reliable estimates of the
 * system's true state even in the presence of measurement noise and uncertainty.
 * <p>
 * The Kalman Filter works by combining two sources of information:
 * <ol>
 *     <li>
 *         <em>Prediction</em>: The filter predicts the current state of the system based on the previous state
 *         and the system's dynamics model.
 *     </li>
 *     <li>
 *         <em>Measurement Update</em>: The filter incorporates the new measurement data and corrects the
 *         predicted state based on the measurement's accuracy and the state's uncertainty.
 *     </li>
 * </ol>
 */
public class KalmanFilter2 {
    double x = 0; // your initial state
    double Q = 0.1; // your model covariance
    double R = 0.4; // your sensor covariance
    double p = 1; // your initial covariance guess
    double K = 1; // your initial Kalman gain guess

    double x_previous = x;
    double p_previous = p;
    double u = 0;
    double z = 0;

    /**
     * @param x the current best estimate of the true state of the dynamic system being tracked or
     *          estimated by the Kalman Filter. It is a vector containing the state variables, such
     *          as position, velocity, orientation, etc., depending on the application. The
     *          prediction step uses the previous state estimate, and the measurement update step
     *          refines this estimate using the new measurement data.
     * @param p the covariance matrix that represents the uncertainty or error in the state estimate (x).
     *          It describes how much each state variable is uncertain. P is typically an n x n matrix,
     *          where n is the number of state variables being estimated. The prediction step updates
     *          the covariance matrix to account for the process noise, while the measurement update
     *          step adjusts it based on the Kalman Gain.
     * @param u the control input or external force applied to the system. In some applications, the
     *          Kalman Filter may consider the effect of external forces that can be directly measured
     *          or estimated to improve state estimation accuracy. The control input is incorporated
     *          into the state transition equation during the prediction step.
     * @param z the actual measurement obtained from sensors or measurements from the environment. It
     *          may include noisy or imperfect readings of the true state of the system. The Kalman Filter
     *          uses this measurement to update the state estimate during the measurement update step.
     * @param Q the covariance matrix that represents the uncertainty or error in the system dynamics
     *          model (process noise). It accounts for unpredicted disturbances or noise in the system,
     *          which the filter assumes to be Gaussian. The prediction step modifies the covariance
     *          matrix to account for the process noise.
     * @param R the covariance matrix that represents the uncertainty or error in the measurements
     *          obtained from sensors (measurement noise). It accounts for the noise present in the
     *          measurements and is typically provided by the sensor manufacturer or determined
     *          experimentally. The measurement update step adjusts the covariance matrix based on
     *          the measurement noise.
     * @param K the Kalman Gain, which is a crucial factor in the measurement update step. It determines
     *          how much the state estimate (x) is adjusted based on the new measurement (z) and is
     *          computed using the covariance matrices P and R. The Kalman Gain balances the impact
     *          of the predicted state and the measurement to achieve optimal state estimation.
     */
    public KalmanFilter2(double x, double p, double u, double z, double Q, double R, double K) {
        this.x = x;
        this.p = p;
        this.u = u;
        this.z = z;
        this.Q = Q;
        this.R = R;
        this.K = K;
        this.x_previous = x;
        this.p_previous = p;
    }

    /**
     * Updates the Kalman Filter with a new measurement.
     *
     * @param measurement the new measurement
     * @return the current best estimate of the true state
     */
    public double update(double measurement) {
        // prediction
        setZ(measurement);
        x = calculateX_prediction(x_previous, u);
        p = calculateP_prediction(p_previous, Q);

        // update
        K = calculateK(p, R);
        x = calculateX(x, K, measurement);
        p = calculateP(K, p);

        // save previous values
        x_previous = x;
        p_previous = p;

        return x;
    }

    /**
     * Gets a prediction for the true state of the system.
     *
     * @param x the previous best estimate for the true state of the system
     * @param u the control input or external force applied to the system
     * @return x+u
     */
    public double calculateX_prediction(double x, double u) {
        return x + u;
    }

    /**
     * Calculates the covariance prediction.
     *
     * @param p the uncertainty or error in the state estimate (x)
     * @param Q the uncertainty or error in the system dynamics model (process noise)
     * @return p+Q
     */
    public double calculateP_prediction(double p, double Q) {
        return p + Q;
    }

    /**
     * Calculates the Kalman Gain.
     *
     * @param p the uncertainty or error in the state estimate (x)
     * @param R the uncertainty or error in the system dynamics model (process noise)
     * @return p/(p+r)
     */
    public static double calculateK(double p, double R) {
        return p / (p + R);
    }

    /**
     * Calculates a new best estimate of the true state of the dynamic system.
     *
     * @param x           the current best estimate of the true state of the dynamic system
     * @param K           the Kalman Gain
     * @param measurement the new measurement
     * @return x+K*(measurement-x)
     */
    public double calculateX(double x, double K, double measurement) {
        return x + K * (measurement - x);
    }

    /**
     * Calculates the covariance matrix that represents the uncertainty or error in the state estimate (x).
     *
     * @param K The Kalman Gain
     * @param p the uncertainty or error in the state estimate (x)
     * @return (1 - k)*p
     */
    public double calculateP(double K, double p) {
        return (1 - K) * p;
    }

    /**
     * Gets the current best estimate of the true state of the dynamic system.
     *
     * @return x the current best estimate of the true state of the dynamic system
     */
    public double getX() {
        return x;
    }

    /**
     * Sets the current best estimate of the true state of the dynamic system.
     *
     * @param x the current best estimate of the true state of the dynamic system
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * Gets the uncertainty or error in the state estimate (x).
     *
     * @return p the uncertainty or error in the state estimate (x)
     */
    public double getP() {
        return p;
    }

    /**
     * Sets the uncertainty or error in the state estimate (x).
     *
     * @param p the uncertainty or error in the state estimate (x)
     */
    public void setP(double p) {
        this.p = p;
    }

    /**
     * Gets the control input or external force applied to the system.
     *
     * @return the control input or external force applied to the system
     */
    public double getU() {
        return u;
    }

    /**
     * Sets the control input or external force applied to the system.
     *
     * @param u the control input or external force applied to the system
     */
    public void setU(double u) {
        this.u = u;
    }

    /**
     * Gets the actual measurement obtained from sensors or measurements from the environment.
     *
     * @return z the actual measurement obtained from sensors or measurements from the environment
     */
    public double getZ() {
        return z;
    }

    /**
     * Sets the actual measurement obtained from sensors or measurements from the environment.
     *
     * @param z the actual measurement obtained from sensors or measurements from the environment
     */
    public void setZ(double z) {
        this.z = z;
    }

    /**
     * Gets the uncertainty or error in the system dynamics model (process noise).
     *
     * @return q Gets the uncertainty or error in the system dynamics model (process noise)
     */
    public double getQ() {
        return Q;
    }

    /**
     * Sets Gets the uncertainty or error in the system dynamics model (process noise).
     *
     * @param Q Gets the uncertainty or error in the system dynamics model (process noise)
     */
    public void setQ(double Q) {
        this.Q = Q;
    }

    /**
     * Gets the uncertainty or error in the measurements obtained from sensors (measurement noise).
     *
     * @return r the uncertainty or error in the measurements obtained from sensors (measurement noise)
     */
    public double getR() {
        return R;
    }

    /**
     * Sets the uncertainty or error in the measurements obtained from sensors (measurement noise).
     *
     * @param R the uncertainty or error in the measurements obtained from sensors (measurement noise)
     */
    public void setR(double R) {
        this.R = R;
    }

    /**
     * Gets the Kalman Gain.
     *
     * @return k the Kalman Gain
     */
    public double getK() {
        return K;
    }

    /**
     * Sets the Kalman Gain.
     *
     * @param K the Kalman Gain
     */
    public void setK(double K) {
        this.K = K;
    }

    /**
     * Gets the previous best estimate of the true state of the dynamic system.
     *
     * @return x_previous the previous best estimate of the true state of the dynamic system
     */
    public double getX_previous() {
        return x_previous;
    }

    /**
     * Sets the previous best estimate of the true state of the dynamic system.
     *
     * @param x_previous the previous best estimate of the true state of the dynamic system
     */
    public void setX_previous(double x_previous) {
        this.x_previous = x_previous;
    }

    /**
     * Gets the previous uncertainty or error in the state estimate (x).
     *
     * @return p_previous the previous uncertainty or error in the state estimate (x)
     */
    public double getP_previous() {
        return p_previous;
    }

    /**
     * Sets the previous uncertainty or error in the state estimate (x).
     *
     * @param p_previous the previous uncertainty or error in the state estimate (x)
     */
    public void setP_previous(double p_previous) {
        this.p_previous = p_previous;
    }
}
