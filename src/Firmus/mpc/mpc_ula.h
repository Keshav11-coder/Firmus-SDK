#pragma once
/**
 *  Two MPC policies aligned with your cascade:
 *
 *  (A) BodyAttitudeMPCPolicy<N>      : x=[theta; omega] ∈ R^6, u=alpha ∈ R^3
 *      Discrete model (small-angle):
 *        theta_{k+1} = theta_k + dt*omega_k + 0.5*dt^2*alpha_k
 *        omega_{k+1} = omega_k + dt*alpha_k
 *      Output: angular acceleration (Vector3) — plug into BodyRateLayer (inner).
 *
 *  (B) BodyOrientationMPCPolicy<N>   : x=theta ∈ R^3, u=omega_ref ∈ R^3
 *      Discrete model:
 *        theta_{k+1} = theta_k + dt*omega_ref_k
 *      Output: angular velocity setpoint (Vector3) — plug into BodyOrientationLayer (outer).
 *
 *  Both assemble the QP:
 *     min  0.5 U^T H U + f^T U
 *     H = 2(Su^T Qbar Su + Rbar),   f = 2 Su^T Qbar (Sx x0 - Xref)
 *
 *  Notes:
 *   - Keep weights diagonal first; add cross-coupling later if needed.
 *   - For production, swap inverse() with a tiny Cholesky solve.
 */

#include <algorithm>
#include <cmath>
#include "../tools/matrix2.h"
#include "../../Firmus.h" // Vector3, Snapshot, IBodyCorrectionPolicy

namespace MPC
{

    // ---- small helpers (block insert, extraction) --------------------------------
    template <int DR, int DC, int R, int C>
    static inline void put_block(FMat::matrix<float, DR, DC> &D,
                                 const FMat::matrix<float, R, C> &B,
                                 int r0, int c0)
    {
        for (int r = 0; r < R; ++r)
            for (int c = 0; c < C; ++c)
                D(r0 + r, c0 + c) = B(r, c);
    }

    template <int N>
    static inline FMat::matrix<float, 3 * N, 1> repeat_ref3(const Vector3 &v)
    {
        auto Xref = FMat::zeros<float, 3 * N, 1>();
        for (int k = 0; k < N; ++k)
        {
            Xref(3 * k + 0, 0) = v.x;
            Xref(3 * k + 1, 0) = v.y;
            Xref(3 * k + 2, 0) = v.z;
        }
        return Xref;
    }

    // ============================================================================
    // (A) FULL-STATE ATTITUDE MPC  (x=[theta;omega], u=alpha)
    // ============================================================================

    template <int N>
    class BodyAttitudeMPCPolicy : public IBodyCorrectionPolicy
    {
        static_assert(N >= 1, "Horizon N must be >= 1");
        // dims
        static constexpr int NX = 6;         // [theta(3); omega(3)]
        static constexpr int NU = 3;         // alpha(3)
        static constexpr int NS = NX * N;    // stacked states: x1..xN
        static constexpr int NUALL = NU * N; // N moves

        // weights
        FMat::matrix<float, NX, NX> Q = FMat::diagonal<float, NX>({10, 10, 10, 1, 1, 1});
        FMat::matrix<float, NU, NU> R = FMat::diagonal<float, NU>({0.1f, 0.1f, 0.1f});
        FMat::matrix<float, NX, NX> Qf = FMat::diagonal<float, NX>({10, 10, 10, 1, 1, 1});

        // bounds on alpha
        Vector3 umin = Vector3(-INFINITY, -INFINITY, -INFINITY);
        Vector3 umax = Vector3(INFINITY, INFINITY, INFINITY);

        // discrete model
        FMat::matrix<float, NX, NX> Ad; // [[I, dt*I],[0,I]]
        FMat::matrix<float, NX, NU> Bd; // [[0.5*dt^2*I],[dt*I]]

        // stacks and cost
        FMat::matrix<float, NS, NX> Sx;
        FMat::matrix<float, NS, NUALL> Su;
        FMat::matrix<float, NS, NS> Qbar;
        FMat::matrix<float, NUALL, NUALL> Rbar;
        FMat::matrix<float, NUALL, NUALL> H;
        FMat::matrix<float, NUALL, 1> f;

    public:
        BodyAttitudeMPCPolicy() { build_fixed_weights(); }

        BodyAttitudeMPCPolicy &weights(const FMat::matrix<float, NX, NX> &Q_,
                                       const FMat::matrix<float, NU, NU> &R_,
                                       const FMat::matrix<float, NX, NX> &Qf_)
        {
            Q = Q_;
            R = R_;
            Qf = Qf_;
            build_fixed_weights();
            return *this;
        }
        BodyAttitudeMPCPolicy &bounds(const Vector3 &umin_, const Vector3 &umax_)
        {
            umin = umin_;
            umax = umax_;
            return *this;
        }

        // Expected to be called by BodyRateLayer OR directly by a controller needing alpha.
        Vector3 compute(const Snapshot &target, const Snapshot &measured, float dt) override
        {
            // Model matrices
            build_Ad_Bd(dt);

            // Build horizon stacks
            build_stacks();

            // Build reference (constant over horizon): we track both theta and omega.
            FMat::matrix<float, NX, 1> x0;
            x0(0, 0) = measured.orientation.x;
            x0(1, 0) = measured.orientation.y;
            x0(2, 0) = measured.orientation.z;
            x0(3, 0) = measured.angular_velocity.x;
            x0(4, 0) = measured.angular_velocity.y;
            x0(5, 0) = measured.angular_velocity.z;

            FMat::matrix<float, NS, 1> Xref = FMat::zeros<float, NS, 1>();
            for (int k = 0; k < N; ++k)
            {
                Xref(k * NX + 0, 0) = target.orientation.x;
                Xref(k * NX + 1, 0) = target.orientation.y;
                Xref(k * NX + 2, 0) = target.orientation.z;
                Xref(k * NX + 3, 0) = target.angular_velocity.x;
                Xref(k * NX + 4, 0) = target.angular_velocity.y;
                Xref(k * NX + 5, 0) = target.angular_velocity.z;
            }

            // QP matrices
            const auto err = (Sx * x0) - Xref; // NS×1
            auto QtSu = Su.transpose() * Qbar; // (NUALL×NS)
            H = (QtSu * Su) * 2.0f + (Rbar * 2.0f);
            f = (QtSu * err) * 2.0f;

            // Solve U* = -H^{-1} f  (add tiny reg)
            auto Hreg = H;
            for (int i = 0; i < NUALL; ++i)
                Hreg(i, i) = Hreg(i, i) + 1e-6f;
            auto Ustar = (Hreg.inverse()) * (f * (-1.0f)); // NUALL×1

            // First move u0 and clamp
            Vector3 u0;
            u0.x = constrain(Ustar(0, 0), umin.x, umax.x);
            u0.y = constrain(Ustar(1, 0), umin.y, umax.y);
            u0.z = constrain(Ustar(2, 0), umin.z, umax.z);
            return u0; // angular acceleration
        }

        void reset() override {}

    private:
        void build_Ad_Bd(float dt)
        {
            auto I3 = FMat::identity<float, 3>();
            auto Z3 = FMat::zeros<float, 3, 3>();

            // Ad = [[I3, dt*I3],[Z3, I3]]
            Ad = FMat::zeros<float, 6, 6>();
            put_block<6, 6, 3, 3>(Ad, I3, 0, 0);
            put_block<6, 6, 3, 3>(Ad, I3 * dt, 0, 3);
            put_block<6, 6, 3, 3>(Ad, I3, 3, 3);

            // Bd = [[0.5*dt^2*I3],[dt*I3]]
            Bd = FMat::zeros<float, 6, 3>();
            put_block<6, 3, 3, 3>(Bd, I3 * (0.5f * dt * dt), 0, 0);
            put_block<6, 3, 3, 3>(Bd, I3 * dt, 3, 0);
        }

        void build_fixed_weights()
        {
            // Qbar = blkdiag(Q,...,Q,Qf)  (N blocks)
            Qbar = FMat::zeros<float, NS, NS>();
            for (int k = 0; k < N - 1; ++k)
                put_block<NS, NS, NX, NX>(Qbar, Q, k * NX, k * NX);
            put_block<NS, NS, NX, NX>(Qbar, Qf, (N - 1) * NX, (N - 1) * NX);

            // Rbar = blkdiag(R,...,R)     (N blocks)
            Rbar = FMat::zeros<float, NUALL, NUALL>();
            for (int k = 0; k < N; ++k)
                put_block<NUALL, NUALL, NU, NU>(Rbar, R, k * NU, k * NU);
        }

        void build_stacks()
        {
            // Sx row blocks are A^1..A^N
            Sx = FMat::zeros<float, NS, NX>();
            auto Ak = Ad; // A^1
            for (int k = 0; k < N; ++k)
            {
                put_block<NS, NX, NX, NX>(Sx, Ak, k * NX, 0);
                Ak = Ak * Ad; // next power
            }

            // Su row k, col j block = A^{k-1-j} B for j<=k, else 0   (k=0..N-1)
            Su = FMat::zeros<float, NS, NUALL>();
            for (int k = 0; k < N; ++k)
            {
                FMat::matrix<float, NX, NX> Aexp = FMat::identity<float, NX>(); // A^0
                for (int j = k; j >= 0; --j)
                {
                    // place A^{k-1-j} B at (row k, col j)
                    put_block<NS, NUALL, NX, 3>(Su, Aexp * Bd, k * NX, j * NU);
                    Aexp = Aexp * Ad; // next power
                }
            }
        }
    };

    // ============================================================================
    // (B) ORIENTATION-ONLY MPC  (x=theta, u=omega_ref)  -- OUTER LOOP
    // ============================================================================

    template <int N>
    class BodyOrientationMPCPolicy : public IBodyCorrectionPolicy
    {
        static_assert(N >= 1, "Horizon N must be >= 1");
        static constexpr int NX = 3; // theta
        static constexpr int NU = 3; // omega_ref
        static constexpr int NS = NX * N;
        static constexpr int NUALL = NU * N;

        // weights
        FMat::matrix<float, NX, NX> Q = FMat::diagonal<float, NX>({10, 10, 10});
        FMat::matrix<float, NU, NU> R = FMat::diagonal<float, NU>({0.5f, 0.5f, 0.5f});
        FMat::matrix<float, NX, NX> Qf = FMat::diagonal<float, NX>({10, 10, 10});

        // bounds on omega_ref
        Vector3 umin = Vector3(-INFINITY, -INFINITY, -INFINITY);
        Vector3 umax = Vector3(INFINITY, INFINITY, INFINITY);

        // discrete model: theta_{k+1} = theta_k + dt * omega_ref_k
        FMat::matrix<float, NX, NX> Ad; // I3
        FMat::matrix<float, NX, NU> Bd; // dt * I3

        // stacks and cost
        FMat::matrix<float, NS, NX> Sx;
        FMat::matrix<float, NS, NUALL> Su;
        FMat::matrix<float, NS, NS> Qbar;
        FMat::matrix<float, NUALL, NUALL> Rbar;
        FMat::matrix<float, NUALL, NUALL> H;
        FMat::matrix<float, NUALL, 1> f;

    public:
        BodyOrientationMPCPolicy()
        {
            build_fixed_weights();
            Ad = FMat::identity<float, 3>();
        }

        BodyOrientationMPCPolicy &weights(const FMat::matrix<float, NX, NX> &Q_,
                                          const FMat::matrix<float, NU, NU> &R_,
                                          const FMat::matrix<float, NX, NX> &Qf_)
        {
            Q = Q_;
            R = R_;
            Qf = Qf_;
            build_fixed_weights();
            return *this;
        }
        BodyOrientationMPCPolicy &bounds(const Vector3 &umin_, const Vector3 &umax_)
        {
            umin = umin_;
            umax = umax_;
            return *this;
        }

        // Expected to be called by BodyOrientationLayer; returns omega_ref (Vector3)
        Vector3 compute(const Snapshot &target, const Snapshot &measured, float dt) override
        {
            Bd = FMat::identity<float, 3>() * dt;

            build_stacks();

            // Initial theta
            FMat::matrix<float, NX, 1> x0;
            x0(0, 0) = measured.orientation.x;
            x0(1, 0) = measured.orientation.y;
            x0(2, 0) = measured.orientation.z;

            // Constant reference theta over horizon
            auto Xref = repeat_ref3<N>(Vector3(target.orientation.x,
                                               target.orientation.y,
                                               target.orientation.z));

            const auto err = (Sx * x0) - Xref;
            auto QtSu = Su.transpose() * Qbar;
            H = (QtSu * Su) * 2.0f + (Rbar * 2.0f);
            f = (QtSu * err) * 2.0f;

            auto Hreg = H;
            for (int i = 0; i < NUALL; ++i)
                Hreg(i, i) += 1e-6f;
            auto Ustar = (Hreg.inverse()) * (f * (-1.0f)); // NUALL×1

            Vector3 u0;
            u0.x = constrain(Ustar(0, 0), umin.x, umax.x);
            u0.y = constrain(Ustar(1, 0), umin.y, umax.y);
            u0.z = constrain(Ustar(2, 0), umin.z, umax.z);
            return u0; // angular velocity setpoint
        }

        void reset() override {}

    private:
        void build_fixed_weights()
        {
            Qbar = FMat::zeros<float, NS, NS>();
            for (int k = 0; k < N - 1; ++k)
                put_block<NS, NS, 6, 6>(Qbar, Q, k * NX, k * NX);
            put_block<NS, NS, 6, 6>(Qbar, Qf, (N - 1) * NX, (N - 1) * NX);

            Rbar = FMat::zeros<float, NUALL, NUALL>();
            for (int k = 0; k < N; ++k)
                put_block<NUALL, NUALL, NU, NU>(Rbar, R, k * NU, k * NU);
        }

        void build_stacks()
        {
            // Sx = [A; A^2; ...; A^N], but A=I3 → each block is I3
            Sx = FMat::zeros<float, NS, NX>();
            for (int k = 0; k < N; ++k)
                put_block<NS, NX, NX, NX>(Sx, Ad, k * NX, 0);

            // Su lower-triangular with Bd on/below diagonal (since A=I)
            Su = FMat::zeros<float, NS, NUALL>();
            for (int row = 0; row < N; ++row)
            {
                for (int col = 0; col <= row; ++col)
                {
                    put_block<NS, NUALL, NX, NU>(Su, Bd, row * NX, col * NU);
                }
            }
        }
    };

} // namespace MPC
