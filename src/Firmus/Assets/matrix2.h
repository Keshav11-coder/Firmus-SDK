#ifndef MATRIX2_H
#define MATRIX2_H

// #include "../../../../../../../Arduino/hardware/arduino/avr/cores/arduino/Arduino.h"
#include <initializer_list>

namespace Matrix2
{
    template <typename T, size_t Rows, size_t Cols>
    struct matrix
    {
        T data[Rows][Cols]{};

        // Was:
        // constexpr matrix(const std::initializer_list<std::initializer_list<T>> &init)
        matrix(const std::initializer_list<std::initializer_list<T>> &init)
        {
            size_t i = 0;
            for (const auto &row : init)
            {
                size_t j = 0;
                for (const auto &val : row)
                {
                    if (i < Rows && j < Cols)
                    {
                        data[i][j] = val;
                    }
                    ++j;
                }
                ++i;
            }
        }

        // Mutable version
        T &operator()(size_t i, size_t j)
        {
            return data[i][j];
        }

        // Const version — was commented out, now corrected and restored
        const T &operator()(size_t i, size_t j) const
        {
            return data[i][j];
        }

        constexpr const size_t columns() const
        {
            return Cols;
        }
        constexpr const size_t rows() const
        {
            return Rows;
        }

        // Row/column access
        constexpr matrix<T, 1, Cols> row(size_t r) const
        {
            matrix<T, 1, Cols> result{};
            for (size_t i = 0; i < Rows; ++i)
            {
                if (i == r)
                {
                    for (size_t j = 0; j < Cols; ++j)
                    {
                        result(i, j) = data[i][j];
                    }
                }
            }
            return result;
        }

        template <typename T_, size_t Rows_, size_t Cols_>
        constexpr matrix<T_, Rows_, 1> column(size_t c) const
        {
            matrix<T_, Rows_, 1> result{};
            for (size_t i = 0; i < Rows_; ++i)
            {
                result(i, 0) = data[i][c];
            }
            return result;
        }

        // Transpose
        constexpr matrix<T, Cols, Rows> transpose() const
        {
            matrix<T, Cols, Rows> result{};
            for (size_t i = 0; i < Rows; ++i)
            {
                for (size_t j = 0; j < Cols; ++j)
                {
                    result(j, i) = data[i][j];
                }
            }
            return result;
        }

        // Addition
        constexpr matrix<T, Rows, Cols> operator+(const matrix<T, Rows, Cols> &other) const
        {
            matrix<T, Rows, Cols> result{};
            for (size_t i = 0; i < Rows; ++i)
            {
                for (size_t j = 0; j < Cols; ++j)
                {
                    result(i, j) = data[i][j] + other(i, j);
                }
            }
            return result;
        }

        // Subtraction
        constexpr matrix<T, Rows, Cols> operator-(const matrix<T, Rows, Cols> &other) const
        {
            matrix<T, Rows, Cols> result{};
            for (size_t i = 0; i < Rows; ++i)
            {
                for (size_t j = 0; j < Cols; ++j)
                {
                    result(i, j) = data[i][j] - other(i, j);
                }
            }
            return result;
        }

        // Element-wise division
        constexpr matrix<T, Rows, Cols> operator/(T factor) const
        {
            matrix<T, Rows, Cols> result{};
            for (size_t i = 0; i < Rows; ++i)
            {
                for (size_t j = 0; j < Cols; ++j)
                {
                    result(i, j) = data[i][j] / factor;
                }
            }
            return result;
        }

        // Lightweight Inverse (only works for square matrices)
        constexpr matrix<T, Rows, Cols> inverse() const
        {
            static_assert(Rows == Cols, "Inverse can only be computed for square matrices.");
            matrix<T, Rows, Cols> result{};
            matrix<T, Rows, Cols> temp = *this;

            // Create an identity matrix
            for (size_t i = 0; i < Rows; ++i)
            {
                result(i, i) = static_cast<T>(1);
            }

            // Perform Gaussian elimination
            for (size_t i = 0; i < Rows; ++i)
            {
                T diag = temp(i, i);
                if (diag == static_cast<T>(0))
                {
                    // Handle singular matrix case (not invertible)
                    return matrix<T, Rows, Cols>{}; // Return zero matrix or throw an error
                }

                for (size_t j = 0; j < Cols; ++j)
                {
                    temp(i, j) /= diag;
                    result(i, j) /= diag;
                }

                for (size_t k = 0; k < Rows; ++k)
                {
                    if (k != i)
                    {
                        T factor = temp(k, i);
                        for (size_t j = 0; j < Cols; ++j)
                        {
                            temp(k, j) -= factor * temp(i, j);
                            result(k, j) -= factor * result(i, j);
                        }
                    }
                }
            }

            return result;
        }

        // Pseudo-Inverse
        constexpr matrix<T, Cols, Rows> pseudoinverse() const
        {
            matrix<T, Rows, Cols> temp = *this;

            if (temp.rows() < temp.columns())
            {
                return temp.transpose() * (temp * temp.transpose()).inverse();
            }
            else if (temp.rows() > temp.columns())
            {
                return (temp.transpose() * temp).inverse() * temp.transpose();
            }
            else
            {
                if (temp.rows() == temp.columns())
                {
                    return temp.inverse();
                }
                else
                {
                    return temp.transpose(); // Return zero matrix or throw an error
                }
            }
        }

        // Vector norm (magnitude), only valid for vectors (Rows==1 or Cols==1)
        constexpr T norm() const
        {
            static_assert(Rows == 1 || Cols == 1, "Norm can only be computed for vectors (1 row or 1 column).");
            T sum = static_cast<T>(0);

            if (Rows == 1)
            {
                for (size_t j = 0; j < Cols; ++j)
                {
                    T val = data[0][j];
                    sum += val * val;
                }
            }
            else // Cols == 1
            {
                for (size_t i = 0; i < Rows; ++i)
                {
                    T val = data[i][0];
                    sum += val * val;
                }
            }

            return sqrt(sum);
        }
    };

    // Matrix multiplication operator
    template <typename T, size_t RowsA, size_t ColsA, size_t ColsB>
    constexpr matrix<T, RowsA, ColsB> operator*(
        const matrix<T, RowsA, ColsA> &a,
        const matrix<T, ColsA, ColsB> &b)
    {
        matrix<T, RowsA, ColsB> result{};

        for (size_t i = 0; i < RowsA; ++i)
        {
            for (size_t j = 0; j < ColsB; ++j)
            {
                T sum{};
                for (size_t k = 0; k < ColsA; ++k)
                {
                    sum += a(i, k) * b(k, j);
                }
                result(i, j) = sum;
            }
        }

        return result;
    }

    // Matrix * Scalar
    template <typename T, size_t Rows, size_t Cols>
    constexpr matrix<T, Rows, Cols> operator*(const matrix<T, Rows, Cols> &m, T scalar)
    {
        matrix<T, Rows, Cols> result{};
        for (size_t i = 0; i < Rows; ++i)
        {
            for (size_t j = 0; j < Cols; ++j)
            {
                result(i, j) = m(i, j) * scalar;
            }
        }
        return result;
    }

    // Matrix print function for debugging
    template <typename T, size_t Rows, size_t Cols>
    void print_matrix(const matrix<T, Rows, Cols> &m)
    {
        Serial.println("{");
        for (size_t i = 0; i < Rows; ++i)
        {
            Serial.print("  {");
            for (size_t j = 0; j < Cols; ++j)
            {
                Serial.print(m(i, j));
                if (j < Cols - 1)
                {
                    Serial.print(", ");
                }
            }
            if (i < Rows - 1)
            {
                Serial.println("},");
            }
            else
            {
                Serial.println("}");
            }
        }
        Serial.println("}");
    }

    template <typename T, size_t Rows, size_t Cols>
    constexpr matrix<T, Rows, Cols> make_matrix(const std::initializer_list<std::initializer_list<T>> &data)
    {
        return matrix<T, Rows, Cols>(data);
    }

    // Matrix3 cross product
    inline matrix<float, 3, 3> cross_matrix3(Matrix2::matrix<float, 3, 1> a)
    {
        return make_matrix<float, 3, 3>({{0, -a(2, 0), a(1, 0)},
                                         {a(2, 0), 0, -a(0, 0)},
                                         {-a(1, 0), a(0, 0), 0}});
    }

    template <typename T, size_t Rows, size_t Cols>
    matrix<T, Rows, Cols> zeros()
    {
        matrix<T, Rows, Cols> result{};
        for (size_t i = 0; i < Rows; ++i)
        {
            for (size_t j = 0; j < Cols; ++j)
            {
                result(i, j) = static_cast<T>(0);
            }
        }
        return result;
    }

    template <typename T, size_t I>
    matrix<T, I, I> identity()
    {
        matrix<T, I, I> result = zeros<T, I, I>();
        for (size_t i = 0; i < I; ++i)
        {
            result(i, i) = static_cast<T>(1);
        }
        return result;
    }

    template <typename T, size_t D>
    matrix<T, D, D> diagonal(const std::initializer_list<T> &data)
    {
        matrix<T, D, D> result = zeros<T, D, D>();
        size_t i = 0;
        for (const auto &val : data)
        {
            if (i < D)
            {
                result(i, i) = val;
                ++i;
            }
        }
        return result;
    }

    template <typename T>
    matrix<T, 3, 3> rotation_x(const T &angle)
    {
        return make_matrix<T, 3, 3>({{1, 0, 0},
                                     {0, cos(angle), -sin(angle)},
                                     {0, sin(angle), cos(angle)}});
    }

    template <typename T>
    matrix<T, 3, 3> rotation_y(const T &angle)
    {
        return make_matrix<T, 3, 3>({{cos(angle), 0, sin(angle)},
                                     {0, 1, 0},
                                     {-sin(angle), 0, cos(angle)}});
    }

    template <typename T>
    matrix<T, 3, 3> rotation_z(const T &angle)
    {
        return make_matrix<T, 3, 3>({{cos(angle), -sin(angle), 0},
                                     {sin(angle), cos(angle), 0},
                                     {0, 0, 1}});
    }

    template <typename T>
    matrix<T, 3, 3> rotation_zyx(const T &roll, const T &pitch, const T &yaw)
    {
        return rotation_z(yaw) * rotation_y(pitch) * rotation_x(roll);
    }
}

#endif