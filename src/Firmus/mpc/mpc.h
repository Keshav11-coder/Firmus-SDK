#pragma once

/** © 2025 Keshav Haripersad
 *  MPC Body Control implementation - mpc.h
 *  | Firmus.h for full virtual frame table/overview..
 *  | This is frame-level control; no mixing or hardware is done here.
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.5+
 */

#include <cmath>

#include "../tools/matrix2.h"

/* Forward declarations from ~/Firmus.h */
struct Vector3;
struct Snapshot;

class IBodyCorrectionPolicy;
class IBodyControlLayer;

/* Model Predictive Control (MPC) namespace (containment) */
namespace MPC
{
    /**
     * @class BodyAttitudeMPCPolicy
     * @brief Implements a discrete model (u = alpha) state-based cost correction using quadratic programming.
     *
     * This class untilizes the IBodyCorrectionPolicy interface by applying a full-state
     * correction, solving a quadratic program (cost) to eventually determine the correct
     * outputs (in this case torques).
     *
     * Model Predictive Control is a full-state, constraint-aware solver. It predicts
     * dynamics over a horizon (say 10 steps) and optimizes the control input (u) -> torque or ang_acc.
     *
     * MPC is computationally heavy, so it should be used with caution in high-frequency loops. It's
     * not often used as a standalone control system, but rather as a hybrid to solve specific parts
     * of a control problem properly, often using better couplings and being more aware of constraints.
     *
     * The ideal use for MPC is in a hybrid with Cascaded Control. When there are usually two loops
     * (fast inner, slow outer) both controller using PID, the slower (outer) loop will be maintained
     * by MPC. MPC (so to say) provides "trajectory points" the fast inner loop should follow.
     *
     * MPC => angles -> returns target ang_vel
     * PID => rates  -> returns target ang_acc
     *
     * Typical body controller can still be used to compute torque from the ang_acc based on the model.
     *
     * MPC is MODEL-SENSITIVE, so slight changes will drastically affect how the system behaves. Ensure close
     * resemblance to the actual plant before productive flight.
     *
     * - Since MPC is computationally heavy, most of the below implementations will be horizon-limited.
     * - Inertia coupling between axes is not yet supported.
     * - Below implementations only cover attitude due to matrix sizes being fixed. This is a common problem
     *   everywhere. Lateral control is not yet supported.
     */
    template <int N>
    class BodyAttitudeMPCPolicy : public IBodyCorrectionPolicy
    {
    private:
        /* Control / Model constants */
        static constexpr int Ad_size = 6;
        static constexpr int Bd_r = 6;
        static constexpr int Bd_c = 3;
        static constexpr int x_size = 6;
        static constexpr int u_size = 3;

        /* Control Knobs / User Pramaters */
        FMat::matrix<float, x_size, x_size> Q = FMat::identity<float, x_size>();
        FMat::matrix<float, u_size, u_size> R = FMat::identity<float, u_size>();
        FMat::matrix<float, x_size, x_size> Qf = Q;

        /* Discrete model */
        FMat::matrix<float, Ad_size, Ad_size> Ad = FMat::identity<float, Ad_size>();
        FMat::matrix<float, Bd_r, Bd_c> Bd = FMat::zeros<float, Bd_r, Bd_c>();

        /* State & Input stacks; X = S_x*x_0 + S_u*U */
        FMat::matrix<float, x_size * N, 1> X = FMat::zeros<float, x_size * N, 1>();
        FMat::matrix<float, u_size * N, 1> U = FMat::zeros<float, u_size * N, 1>();

        FMat::matrix<float, Ad_size * N, Ad_size> Sx = FMat::zeros<float, Ad_size * N, Ad_size>();
        FMat::matrix<float, Bd_r * N, Bd_c * N> Su = FMat::zeros<float, Bd_r * N, Bd_c * N>();

        /* [?] Other */
        float _dt = 0.01;

    public:
        /** Design Parameters (Control Knobs)
         * @param Q controls how hard the optimizer tries to reduce state error (x)
         *  - Bigger => optimizer tries harder
         *  - Smaller => optimizer is more relaxed
         *  - Scales with state size (e.g. x = 6*1, Q = 6*6)
         *
         * @param R penalizes input effort
         *  - Bigger => optimizer will utilize small control actions
         *  - Smaller => optimizer is willing to use aggressive accelerations
         *  - Scales with control input size (e.g. u = 3*1, R = 3*3)
         *
         * @param Qf (terminal weight) adds extra emphasis on ending close to the reference after N steps
         *  - Not often used, often simply set to Q but useful as bias.
         *
         * Hardcoded as x(6), u(3). This is minimal full-state attitude MPC.
         *
         * Motivates using diagonal matrices (to penalize independent axes/channels),
         * but coupled penalizing is also supported (off-diagonal elements).
         */
        BodyAttitudeMPCPolicy(FMat::matrix<float, 6, 6> Q, FMat::matrix<float, 3, 3> R, FMat::matrix<float, 6, 6> Qf) : Q(Q), R(R), Qf(Qf)
        {
            Ad.set_block<3, 3>(0, 3, FMat::identity<float, 3>() * _dt);

            const float scalar = (0.5 * (_dt * _dt));
            Bd.set_block<3, 3>(0, 0, FMat::identity<float, 3>() * scalar); // [[0.5*dt*I3]]
            Bd.set_block<3, 3>(3, 0, FMat::identity<float, 3>() * _dt);    // [[dt*I3]]

            // Form Sx
            FMat::matrix<float, x_size, x_size> A_pow = FMat::identity<float, x_size>();
            for (int i = 0; i < N; i++)
            {
                A_pow = A_pow * Ad; // Stays Ad at the first iteration, but next iterations use Ad * Ad ... * Ad

                Sx.template set_block<x_size, x_size>(x_size * i, 0, A_pow);
            }

            // Form Su
            /**
             * Strategy:
             * For every N, the equation stacks, e.g.:
             * x1 = Ax0 + Bu0
             * x2 = (A^2)x0 + ABu0 + Bu1
             * x3 = (A^3)x0 + (A^2)Bu0 + ABu1 + Bu3
             *
             * This can be interpreted in matrix form (block diagonals, all upper-right is 0).
             */
            for (int i = 0; i < N; i++) // row block
            {
                for (int j = 0; j <= i; j++) // col block
                {
                    // exponent = i - j
                    auto Apow = Ad.pow(i - j); // implement small helper for A^k
                    auto block = Apow * Bd;    // Bd = B

                    // place block into Su at (i,j)
                    Su.template set_block<Bd_r, Bd_c>(
                        Bd_r * i, // row offset
                        Bd_c * j, // col offset
                        block);
                }
            }
        }

        FMat::matrix<float, 6 * N, 6> getSx()
        {
            return Sx;
        }

        FMat::matrix<float, 6 * N, 3 * N> getSu()
        {
            return Su;
        }

        Snapshot compute(const Snapshot &target, const Snapshot &measured, float dt) override
        {
            // ------------------------------------------------------------
            // 1) Build x0 and x_ref (6x1 each) from Snapshots
            // ------------------------------------------------------------
            // Assume Snapshot exposes orientation (theta), angular_velocity (omega)
            // and you can read them into floats; adapt as needed to your types.
            FMat::matrix<float, 6, 1> x0 = FMat::zeros<float, 6, 1>();
            FMat::matrix<float, 6, 1> xref = FMat::zeros<float, 6, 1>();
            x0(0, 0) = measured.orientation.x;      // θx
            x0(1, 0) = measured.orientation.y;      // θy
            x0(2, 0) = measured.orientation.z;      // θz
            x0(3, 0) = measured.angular_velocity.x; // ωx
            x0(4, 0) = measured.angular_velocity.y; // ωy
            x0(5, 0) = measured.angular_velocity.z; // ωz

            xref(0, 0) = target.orientation.x;
            xref(1, 0) = target.orientation.y;
            xref(2, 0) = target.orientation.z;
            xref(3, 0) = target.angular_velocity.x;
            xref(4, 0) = target.angular_velocity.y;
            xref(5, 0) = target.angular_velocity.z;

            // ------------------------------------------------------------
            // 2) Stack X_ref = kron(ones_N, xref)  (size 6N x 1)
            // ------------------------------------------------------------
            FMat::matrix<float, 6 * N, 1> Xref = FMat::zeros<float, 6 * N, 1>();
            for (int i = 0; i < N; ++i)
            {
                FMat::matrix<float, 6, 1> xref_step = xref;

                // Optionally: ramp orientation instead of hard setpoint
                float alpha = float(i + 1) / float(N); // goes 0..1
                xref_step(0, 0) *= alpha;              // pitch_ref
                xref_step(1, 0) *= alpha;              // roll_ref
                xref_step(2, 0) *= alpha;              // yaw_ref

                Xref.template set_block<6, 1>(6 * i, 0, xref_step);
            }

            // ------------------------------------------------------------
            // 3) Build barQ (6N x 6N) and barR (3N x 3N)
            //    barQ = blkdiag(Q, ..., Q, Qf), barR = blkdiag(R, ..., R)
            // ------------------------------------------------------------
            FMat::matrix<float, 6 * N, 6 * N> barQ = FMat::zeros<float, 6 * N, 6 * N>();
            for (int i = 0; i < N - 1; ++i)
            {
                barQ.template set_block<6, 6>(6 * i, 6 * i, Q);
            }
            barQ.template set_block<6, 6>(6 * (N - 1), 6 * (N - 1), Qf);

            FMat::matrix<float, 3 * N, 3 * N> barR = FMat::zeros<float, 3 * N, 3 * N>();
            for (int i = 0; i < N; ++i)
            {
                barR.template set_block<3, 3>(3 * i, 3 * i, R);
            }

            // ------------------------------------------------------------
            // 4) Assemble H and f
            //    H = 2 (Su^T * barQ * Su + barR)
            //    f = 2 Su^T * barQ * (Sx*x0 - Xref)
            // ------------------------------------------------------------
            auto Sx_x0 = Sx * x0;               // (6N x 6)*(6 x 1) => (6N x 1)
            auto err = Sx_x0 - Xref;            // (6N x 1)
            auto QtSu = (barQ * Su);            // (6N x 6N)*(6N x 3N) => (6N x 3N)
            auto SuTQt = Su.transpose() * barQ; // (3N x 6N)

            FMat::matrix<float, 3 * N, 3 * N> H = Su.transpose() * QtSu; // (3N x 6N)*(6N x 3N) => (3N x 3N)
            H = H + barR;                                                // (3N x 3N)
            H = H * 2.0f;

            FMat::matrix<float, 3 * N, 1> f = Su.transpose() * (barQ * err); // (3N x 6N)*(6N x 1) => (3N x 1)
            f = f * 2.0f;

            // (Optional) regularization for numerical robustness:
            // H += eps * I
            {
                const float eps = 1e-8f;
                for (int i = 0; i < 3 * N; ++i)
                    H(i, i) += eps;
            }

            // ------------------------------------------------------------
            // 5) Solve the unconstrained QP: U* = -H^{-1} f
            //    (Replace with a real QP solver if you add constraints)
            // ------------------------------------------------------------
            FMat::matrix<float, 3 * N, 1> Ustar = FMat::zeros<float, 3 * N, 1>();
            {
                // naive solve via inverse (ok for demonstration)
                auto Hinv = H.inverse();        // (3N x 3N)
                Ustar = (Hinv * (f * (-1.0f))); // (3N x 3N)*(3N x 1)
            }

            // ------------------------------------------------------------
            // 6) Extract u0* = first 3 entries of Ustar (angular acceleration)
            // ------------------------------------------------------------
            Vector3 alpha0;
            alpha0.x = Ustar(0, 0);
            alpha0.y = Ustar(1, 0);
            alpha0.z = Ustar(2, 0);

            // ------------------------------------------------------------
            // 7) Write result Snapshot: set angular_acceleration = alpha0.
            //    (Leave torque computation to your BodyController / rigid model.)
            // ------------------------------------------------------------
            Snapshot result = measured; // start from measured, then overwrite the fields you emit
            result.angular_acceleration = alpha0;

            // (Optional) also fill predicted next state if your Snapshot supports it:
            // x1 = Ad*x0 + Bd*u0*
            // auto x1 = Ad * x0 + Bd * FMat::from_vec3(alpha0);
            // result.predicted_orientation = ...
            // result.predicted_angular_velocity = ...

            return result;
        }

        void reset() override
        {
        }

        /* Stubs */
        Vector3 compute(const Vector3 &target, const Vector3 &measured, float dt) override { return Vector3(0, 0, 0); };
    };

    /**
     * @class BodyOrientationMPCPolicy
     * @brief Implements a discrete model (u = rate) state-based cost correction using quadratic programming.
     *
     * This class untilizes the IBodyCorrectionPolicy interface by applying an orientation-only
     * correction, solving a quadratic program (cost) to eventually determine the correct
     * outputs (in this case angular velocities).
     *
     * Model Predictive Control is a full-state, constraint-aware solver. It predicts
     * dynamics over a horizon (say 10 steps) and optimizes the control input (u) -> torque or ang_acc, but in
     * this case rates, because it only predicts angles.
     *
     * MPC is computationally heavy, so it should be used with caution in high-frequency loops. It's
     * not often used as a standalone control system, but rather as a hybrid to solve specific parts
     * of a control problem properly, often using better couplings and being more aware of constraints.
     *
     * The ideal use for MPC is in a hybrid with Cascaded Control. When there are usually two loops
     * (fast inner, slow outer) both controller using PID, the slower (outer) loop will be maintained
     * by MPC. MPC (so to say) provides "trajectory points" the fast inner loop should follow. This class
     * encourages this heavily, and provides an interface for the outer (slow) loop to calculate angles. This is
     * more efficient and a common hybrid combination.
     *
     * MPC => angles -> returns target ang_vel
     * PID => rates  -> returns target ang_acc
     *
     * Typical body controller can still be used to compute torque from the ang_acc based on the model.
     *
     * MPC is MODEL-SENSITIVE, so slight changes will drastically affect how the system behaves. Ensure close
     * resemblance to the actual plant before productive flight.
     *
     * - Inertia coupling between axes is not yet supported.
     * - Below implementations only cover attitude due to matrix sizes being fixed. This is a common problem
     *   everywhere. Lateral control is not yet supported.
     */
    template <int N>
    class BodyOrientationMPCPolicy : public IBodyCorrectionPolicy
    {
    private:
        /* Control / Model constants */
        static constexpr int Ad_size = 3;
        static constexpr int Bd_r = 3;
        static constexpr int Bd_c = 3;
        static constexpr int x_size = 3;
        static constexpr int u_size = 3;

        /* Control Knobs / User Pramaters */
        FMat::matrix<float, x_size, x_size> Q = FMat::identity<float, x_size>();
        FMat::matrix<float, u_size, u_size> R = FMat::identity<float, u_size>();
        FMat::matrix<float, x_size, x_size> Qf = Q;

        /* Discrete model */
        FMat::matrix<float, Ad_size, Ad_size> Ad = FMat::identity<float, Ad_size>();
        FMat::matrix<float, Bd_r, Bd_c> Bd = FMat::zeros<float, Bd_r, Bd_c>();

        /* State & Input stacks; X = S_x*x_0 + S_u*U */
        FMat::matrix<float, x_size * N, 1> X = FMat::zeros<float, x_size * N, 1>();
        FMat::matrix<float, u_size * N, 1> U = FMat::zeros<float, u_size * N, 1>();

        FMat::matrix<float, Ad_size * N, Ad_size> Sx = FMat::zeros<float, Ad_size * N, Ad_size>();
        FMat::matrix<float, Bd_r * N, Bd_c * N> Su = FMat::zeros<float, Bd_r * N, Bd_c * N>();

        /* [?] Other */
        float _dt = 0.01;

    public:
        /** Design Parameters (Control Knobs)
         * @param Q controls how hard the optimizer tries to reduce state error (x)
         *  - Bigger => optimizer tries harder
         *  - Smaller => optimizer is more relaxed
         *  - Scales with state size (e.g. x = 6*1, Q = 6*6)
         *
         * @param R penalizes input effort
         *  - Bigger => optimizer will utilize small control actions
         *  - Smaller => optimizer is willing to use aggressive accelerations
         *  - Scales with control input size (e.g. u = 3*1, R = 3*3)
         *
         * @param Qf (terminal weight) adds extra emphasis on ending close to the reference after N steps
         *  - Not often used, often simply set to Q but useful as bias.
         *
         * Hardcoded as x(6), u(3). This is minimal full-state attitude MPC.
         *
         * Motivates using diagonal matrices (to penalize independent axes/channels),
         * but coupled penalizing is also supported (off-diagonal elements).
         */
        BodyOrientationMPCPolicy(FMat::matrix<float, 3, 3> Q, FMat::matrix<float, 3, 3> R, FMat::matrix<float, 3, 3> Qf) : Q(Q), R(R), Qf(Qf)
        {
            Ad = FMat::identity<float, 3>();

            const float scalar = (0.5 * (_dt * _dt));
            Bd = FMat::identity<float, 3>() * scalar; // [[dt*I3]]

            // Form Sx (Since Ad = I3, Sx is just a tall stack of I3; using pow is unnecessary)
            FMat::matrix<float, x_size, x_size> A_pow = FMat::identity<float, x_size>();
            for (int i = 0; i < N; i++)
            {
                Sx.template set_block<x_size, x_size>(x_size * i, 0, A_pow);
            }

            // Form Su (same thing, block diagonal of dt*I3)
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    Su.template set_block<Bd_r, Bd_c>(Bd_r * i, Bd_c * j, Bd);
                }
            }
        }

        FMat::matrix<float, 3 * N, 3> getSx()
        {
            return Sx;
        }

        FMat::matrix<float, 3 * N, 3 * N> getSu()
        {
            return Su;
        }

        Snapshot compute(const Snapshot &target, const Snapshot &measured, float dt) override
        {
            // ------------------------------------------------------------
            // 1) Build x0 and x_ref (6x1 each) from Snapshots
            // ------------------------------------------------------------
            // Assume Snapshot exposes orientation (theta), angular_velocity (omega)
            // and you can read them into floats; adapt as needed to your types.
            FMat::matrix<float, 3, 1> x0 = FMat::zeros<float, 3, 1>();
            FMat::matrix<float, 3, 1> xref = FMat::zeros<float, 3, 1>();
            x0(0, 0) = measured.orientation.x;      // θx
            x0(1, 0) = measured.orientation.y;      // θy
            x0(2, 0) = measured.orientation.z;      // θz

            xref(0, 0) = target.orientation.x;
            xref(1, 0) = target.orientation.y;
            xref(2, 0) = target.orientation.z;

            // ------------------------------------------------------------
            // 2) Stack X_ref = kron(ones_N, xref)  (size 6N x 1)
            // ------------------------------------------------------------
            FMat::matrix<float, 3 * N, 1> Xref = FMat::zeros<float, 3 * N, 1>();
            for (int i = 0; i < N; ++i)
            {
                FMat::matrix<float, 3, 1> xref_step = xref;

                // Optionally: ramp orientation instead of hard setpoint
                float alpha = float(i + 1) / float(N); // goes 0..1
                xref_step(0, 0) *= alpha;              // pitch_ref
                xref_step(1, 0) *= alpha;              // roll_ref
                xref_step(2, 0) *= alpha;              // yaw_ref

                Xref.template set_block<3, 1>(3 * i, 0, xref_step);
            }

            // ------------------------------------------------------------
            // 3) Build barQ (6N x 6N) and barR (3N x 3N)
            //    barQ = blkdiag(Q, ..., Q, Qf), barR = blkdiag(R, ..., R)
            // ------------------------------------------------------------
            FMat::matrix<float, 3 * N, 3 * N> barQ = FMat::zeros<float, 3 * N, 3 * N>();
            for (int i = 0; i < N - 1; ++i)
            {
                barQ.template set_block<3, 3>(3 * i, 3 * i, Q);
            }
            barQ.template set_block<3, 3>(3 * (N - 1), 3 * (N - 1), Qf);

            FMat::matrix<float, 3 * N, 3 * N> barR = FMat::zeros<float, 3 * N, 3 * N>();
            for (int i = 0; i < N; ++i)
            {
                barR.template set_block<3, 3>(3 * i, 3 * i, R);
            }

            // ------------------------------------------------------------
            // 4) Assemble H and f
            //    H = 2 (Su^T * barQ * Su + barR)
            //    f = 2 Su^T * barQ * (Sx*x0 - Xref)
            // ------------------------------------------------------------
            auto Sx_x0 = Sx * x0;               // (6N x 6)*(6 x 1) => (6N x 1)
            auto err = Sx_x0 - Xref;            // (6N x 1)
            auto QtSu = (barQ * Su);            // (6N x 6N)*(6N x 3N) => (6N x 3N)
            auto SuTQt = Su.transpose() * barQ; // (3N x 6N)

            FMat::matrix<float, 3 * N, 3 * N> H = Su.transpose() * QtSu; // (3N x 6N)*(6N x 3N) => (3N x 3N)
            H = H + barR;                                                // (3N x 3N)
            H = H * 2.0f;

            FMat::matrix<float, 3 * N, 1> f = Su.transpose() * (barQ * err); // (3N x 6N)*(6N x 1) => (3N x 1)
            f = f * 2.0f;

            // (Optional) regularization for numerical robustness:
            // H += eps * I
            {
                const float eps = 1e-8f;
                for (int i = 0; i < 3 * N; ++i)
                    H(i, i) += eps;
            }

            // ------------------------------------------------------------
            // 5) Solve the unconstrained QP: U* = -H^{-1} f
            //    (Replace with a real QP solver if you add constraints)
            // ------------------------------------------------------------
            FMat::matrix<float, 3 * N, 1> Ustar = FMat::zeros<float, 3 * N, 1>();
            {
                // naive solve via inverse (ok for demonstration)
                auto Hinv = H.inverse();        // (3N x 3N)
                Ustar = (Hinv * (f * (-1.0f))); // (3N x 3N)*(3N x 1)
            }

            // ------------------------------------------------------------
            // 6) Extract u0* = first 3 entries of Ustar (angular acceleration)
            // ------------------------------------------------------------
            Vector3 omega0;
            omega0.x = Ustar(0, 0);
            omega0.y = Ustar(1, 0);
            omega0.z = Ustar(2, 0);

            // ------------------------------------------------------------
            // 7) Write result Snapshot: set angular_velocity = omega0.
            //    (Leave rate -> torque to next step, ideally PID.)
            // ------------------------------------------------------------
            Snapshot result = measured; // start from measured, then overwrite the fields you emit
            result.angular_velocity = omega0;

            // (Optional) also fill predicted next state if your Snapshot supports it:
            // x1 = Ad*x0 + Bd*u0*
            // auto x1 = Ad * x0 + Bd * FMat::from_vec3(omega0);
            // result.predicted_orientation = ...
            // result.predicted_angular_velocity = ...

            return result;
        }

        void reset() override
        {
        }

        /* Stubs */
        Vector3 compute(const Vector3 &target, const Vector3 &measured, float dt) override { return Vector3(0, 0, 0); };
    };
}