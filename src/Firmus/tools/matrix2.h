#pragma once

#include <initializer_list>

namespace Matrix2
{
    template <typename T, size_t Rows, size_t Cols>
    class matrix
    {
    private:
        T data[Rows][Cols]{};

    public:
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

        // Block getter
        template <size_t BlockRows, size_t BlockCols>
        constexpr matrix<T, BlockRows, BlockCols> block(size_t row_start, size_t col_start, size_t row_end, size_t col_end) const
        {
            matrix<T, BlockRows, BlockCols> result{};

            for (size_t i = 0; i < BlockRows; ++i)
            {
                for (size_t j = 0; j < BlockCols; ++j)
                {
                    result(i, j) = data[row_start + i][col_start + j];
                }
            }

            return result;
        }

        // Block setter
        template <size_t BlockRows, size_t BlockCols>
        constexpr void set_block(size_t row_start, size_t col_start, const matrix<T, BlockRows, BlockCols> &block_data)
        {
            for (size_t i = 0; i < BlockRows; ++i)
            {
                for (size_t j = 0; j < BlockCols; ++j)
                {
                    data[row_start + i][col_start + j] = block_data(i, j);
                }
            }
        }

        constexpr matrix<T, Rows, Cols> pow(unsigned int n) const
        {
            static_assert(Rows == Cols, "Matrix must be square to raise to a power.");

            // Start with identity matrix using manual construction
            matrix<T, Rows, Cols> result = *this; // Just to get something initialized

            // Now set `result` to identity
            for (size_t i = 0; i < Rows; ++i)
            {
                for (size_t j = 0; j < Cols; ++j)
                {
                    result(i, j) = (i == j) ? static_cast<T>(1) : static_cast<T>(0);
                }
            }

            matrix<T, Rows, Cols> base = *this;

            while (n > 0)
            {
                if (n & 1)
                {
                    result = result * base;
                }
                base = base * base;
                n >>= 1;
            }

            return result;
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
    void print(const matrix<T, Rows, Cols> &m)
    {
        Serial.println("{");
        for (size_t i = 0; i < Rows; ++i)
        {
            Serial.print("  {");
            for (size_t j = 0; j < Cols; ++j)
            {
                Serial.print(m(i, j), 6);
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

    template <typename T, size_t D>
    matrix<T, D, D> diagonal(T data)
    {
        matrix<T, D, D> result = zeros<T, D, D>();
        for (int i = 0; i < D; i++)
        {
            result(i, i) = data;
        }
        return result;
    }

    // Block diagonal constructor
    template <typename T, size_t RowsA, size_t ColsA, size_t RowsB, size_t ColsB>
    constexpr matrix<T, RowsA + RowsB, ColsA + ColsB>
    block_diag(const matrix<T, RowsA, ColsA> &A, const matrix<T, RowsB, ColsB> &B)
    {
        matrix<T, RowsA + RowsB, ColsA + ColsB> M{};
        // Top-left block
        for (size_t i = 0; i < RowsA; i++)
            for (size_t j = 0; j < ColsA; j++)
                M(i, j) = A(i, j);
        // Bottom-right block
        for (size_t i = 0; i < RowsB; i++)
            for (size_t j = 0; j < ColsB; j++)
                M(RowsA + i, ColsA + j) = B(i, j);
        return M;
    }

    // Kronecker product (needed for horizon stacking with I3 ⊗ blocks)
    template <typename T, size_t RowsA, size_t ColsA, size_t RowsB, size_t ColsB>
    constexpr matrix<T, RowsA * RowsB, ColsA * ColsB>
    kron(const matrix<T, RowsA, ColsA> &A, const matrix<T, RowsB, ColsB> &B)
    {
        matrix<T, RowsA * RowsB, ColsA * ColsB> M{};
        for (size_t i = 0; i < RowsA; i++)
            for (size_t j = 0; j < ColsA; j++)
                for (size_t bi = 0; bi < RowsB; bi++)
                    for (size_t bj = 0; bj < ColsB; bj++)
                        M(i * RowsB + bi, j * ColsB + bj) = A(i, j) * B(bi, bj);
        return M;
    }
}

// Alternate name
namespace FMat = Matrix2;