#pragma once

#include "lvins_common/logger.h"
#include "lvins_common/yaml/yaml_serialization.h"

#include <Eigen/Core>
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include <yaml-cpp/yaml.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace YAML {

/**
 * @brief YAML序列化中Eigen::Matrix的实现
 * @details 该类实现了yaml-cpp库中Eigen::Matrix的序列化与反序列化的接口<br/>
 *          用法示例：<br/>
 *          1. 读取（反序列化）：auto matrix = YAML::get<Eigen::MatrixXd>(node, "matrix");<br/>
 *          2. 写入（序列化）：node["matrix"] = matrix;<br/>
 *          支持静态矩阵与动态矩阵
 * @tparam Scalar_ 矩阵元素类型
 * @tparam Rows_ 矩阵行数
 * @tparam Cols_ 矩阵列数
 * @tparam Options_ 矩阵选项
 * @tparam MaxRows_ 矩阵最大行数
 * @tparam MaxCols_ 矩阵最大列数
 */
template<class Scalar_, int Rows_, int Cols_, int Options_, int MaxRows_, int MaxCols_>
struct convert<Eigen::Matrix<Scalar_, Rows_, Cols_, Options_, MaxRows_, MaxCols_>> {
    /**
     * @brief 序列化Eigen::Matrix
     * @tparam Scalar 矩阵元素类型
     * @tparam Rows 矩阵行数
     * @tparam Cols 矩阵列数
     * @tparam Options 矩阵选项
     * @tparam MaxRows 矩阵最大行数
     * @tparam MaxCols 矩阵最大列数
     * @param matrix 矩阵
     * @return 矩阵序列化所在的YAML节点
     */
    template<class Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
    static Node encode(const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> &matrix) {
        Node node;
        typedef typename Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>::Index IndexType;
        IndexType rows = matrix.rows();
        IndexType cols = matrix.cols();
        node["rows"]   = rows;
        node["cols"]   = cols;
        LVINS_CHECK(rows > 0, "The matrix rows should be greater than 0!");
        LVINS_CHECK(cols > 0, "The matrix cols should be greater than 0!");
        for (IndexType i = 0; i < rows; ++i) {
            for (IndexType j = 0; j < cols; ++j) {
                node["data"].push_back(matrix(i, j));
            }
        }
        return node;
    }

    /**
     * @brief 反序列化Eigen::Matrix（静态矩阵）
     * @tparam Scalar 矩阵元素类型
     * @tparam Rows 矩阵行数
     * @tparam Cols 矩阵列数
     * @tparam Options 矩阵选项
     * @tparam MaxRows 矩阵最大行数
     * @tparam MaxCols 矩阵最大列数
     * @param node 矩阵序列化所在的YAML节点
     * @param matrix 矩阵
     * @return 是否反序列化成功
     */
    template<class Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
    static bool decode(const Node &node, Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> &matrix) {
        LVINS_CHECK(node.IsMap(), "Unable to parse the matrix because the node is not a map!");

        typedef typename Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>::Index IndexType;
        const auto rows = YAML::get<IndexType>(node, "rows");
        const auto cols = YAML::get<IndexType>(node, "cols");

        LVINS_CHECK(rows > 0, "The matrix rows should be greater than 0!");
        LVINS_CHECK(cols > 0, "The matrix cols should be greater than 0!");
        LVINS_CHECK(rows == Rows && cols == Cols, "The matrix has wrong size (rows, cols). Wanted: ({},{}) got ({},{})",
                    Rows, Cols, rows, cols);

        [[maybe_unused]] const size_t expected_size = matrix.rows() * matrix.cols();
        LVINS_CHECK(node["data"].IsSequence(), "The matrix data is not a sequence.");
        LVINS_CHECK(node["data"].size() == expected_size, "The data sequence has wrong size. Wanted: {}, got: {}",
                    expected_size, node["data"].size());

        YAML::const_iterator it           = node["data"].begin();
        const YAML::const_iterator it_end = node["data"].end();
        for (IndexType i = 0; i < rows; ++i) {
            for (IndexType j = 0; j < cols; ++j) {
                matrix(i, j) = it->as<Scalar>();
                ++it;
            }
        }

        return true;
    }

    /**
     * @brief 反序列化Eigen::Matrix（动态矩阵，仅行为动态）
     * @tparam Scalar 矩阵元素类型
     * @tparam Cols 矩阵列数
     * @tparam Options 矩阵选项
     * @tparam MaxRows 矩阵最大行数
     * @tparam MaxCols 矩阵最大列数
     * @param node 矩阵序列化所在的YAML节点
     * @param matrix 矩阵
     * @return 是否反序列化成功
     */
    template<class Scalar, int Cols, int Options, int MaxRows, int MaxCols>
    static bool decode(const Node &node,
                       Eigen::Matrix<Scalar, Eigen::Dynamic, Cols, Options, MaxRows, MaxCols> &matrix) {
        LVINS_CHECK(node.IsMap(), "Unable to parse the matrix because the node is not a map!");

        typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Cols, Options, MaxRows, MaxCols>::Index IndexType;
        const auto rows = YAML::get<IndexType>(node, "rows");
        const auto cols = YAML::get<IndexType>(node, "cols");

        LVINS_CHECK(rows > 0, "The matrix rows should be greater than 0!");
        LVINS_CHECK(cols > 0, "The matrix cols should be greater than 0!");
        LVINS_CHECK(cols == Cols, "The matrix has wrong size (rows, cols). Wanted: ({},{}) got ({},{})", rows, Cols,
                    rows, cols);

        matrix.resize(rows, Eigen::NoChange);

        [[maybe_unused]] const size_t expected_size = matrix.rows() * matrix.cols();
        LVINS_CHECK(node["data"].IsSequence(), "The matrix data is not a sequence.");
        LVINS_CHECK(node["data"].size() == expected_size, "The data sequence has wrong size. Wanted: {}, got: {}",
                    expected_size, node["data"].size());

        YAML::const_iterator it           = node["data"].begin();
        const YAML::const_iterator it_end = node["data"].end();
        for (IndexType i = 0; i < rows; ++i) {
            for (IndexType j = 0; j < cols; ++j) {
                matrix(i, j) = it->as<Scalar>();
                ++it;
            }
        }

        return true;
    }

    /**
     * @brief 反序列化Eigen::Matrix（动态矩阵，仅列为动态）
     * @tparam Scalar 矩阵元素类型
     * @tparam Rows 矩阵行数
     * @tparam Options 矩阵选项
     * @tparam MaxRows 矩阵最大行数
     * @tparam MaxCols 矩阵最大列数
     * @param node 矩阵序列化所在的YAML节点
     * @param matrix 矩阵
     * @return 是否反序列化成功
     */
    template<class Scalar, int Rows, int Options, int MaxRows, int MaxCols>
    static bool decode(const Node &node,
                       Eigen::Matrix<Scalar, Rows, Eigen::Dynamic, Options, MaxRows, MaxCols> &matrix) {
        LVINS_CHECK(node.IsMap(), "Unable to parse the matrix because the node is not a map!");

        typedef typename Eigen::Matrix<Scalar, Rows, Eigen::Dynamic, Options, MaxRows, MaxCols>::Index IndexType;
        const auto rows = YAML::get<IndexType>(node, "rows");
        const auto cols = YAML::get<IndexType>(node, "cols");

        LVINS_CHECK(rows > 0, "The matrix rows should be greater than 0!");
        LVINS_CHECK(cols > 0, "The matrix cols should be greater than 0!");
        LVINS_CHECK(rows == Rows, "The matrix has wrong size (rows, cols). Wanted: ({},{}) got ({},{})", Rows, cols,
                    rows, cols);

        matrix.resize(Eigen::NoChange, cols);

        const size_t expected_size = matrix.rows() * matrix.cols();
        LVINS_CHECK(node["data"].IsSequence(), "The matrix data is not a sequence.");
        LVINS_CHECK(node["data"].size() == expected_size, "The data sequence has wrong size. Wanted: {}, got: {}",
                    expected_size, node["data"].size());

        YAML::const_iterator it           = node["data"].begin();
        const YAML::const_iterator it_end = node["data"].end();
        for (IndexType i = 0; i < rows; ++i) {
            for (IndexType j = 0; j < cols; ++j) {
                matrix(i, j) = it->as<Scalar>();
                ++it;
            }
        }

        return true;
    }

    /**
     * @brief 反序列化Eigen::Matrix（动态矩阵，行列均为动态）
     * @tparam Scalar 矩阵元素类型
     * @tparam Options 矩阵选项
     * @tparam MaxRows 矩阵最大行数
     * @tparam MaxCols 矩阵最大列数
     * @param node 矩阵序列化所在的YAML节点
     * @param matrix 矩阵
     * @return 是否反序列化成功
     */
    template<class Scalar, int Options, int MaxRows, int MaxCols>
    static bool decode(const Node &node,
                       Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options, MaxRows, MaxCols> &matrix) {
        LVINS_CHECK(node.IsMap(), "Unable to parse the matrix because the node is not a map.");

        typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options, MaxRows, MaxCols>::Index
                IndexType;
        const auto rows = YAML::get<IndexType>(node, "rows");
        const auto cols = YAML::get<IndexType>(node, "cols");

        LVINS_CHECK(rows > 0, "The matrix rows should be greater than 0!");
        LVINS_CHECK(cols > 0, "The matrix cols should be greater than 0!");
        matrix.resize(rows, cols);

        const size_t expected_size = matrix.rows() * matrix.cols();
        LVINS_CHECK(node["data"].IsSequence(), "The matrix data is not a sequence.");
        LVINS_CHECK(node["data"].size() == expected_size, "The data sequence has wrong size. Wanted: {}, got: {}",
                    expected_size, node["data"].size());

        YAML::const_iterator it           = node["data"].begin();
        const YAML::const_iterator it_end = node["data"].end();
        for (IndexType i = 0; i < rows; ++i) {
            for (IndexType j = 0; j < cols; ++j) {
                matrix(i, j) = it->as<Scalar>();
                ++it;
            }
        }

        return true;
    }
};

} // namespace YAML
