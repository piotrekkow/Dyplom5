#pragma once

#include <algorithm>
#include <common/matrix.hpp>
#include <format>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace logs {
template <typename Id, typename Item, typename LabelFn, typename ValueFn>
void printMatrix(std::string_view title, const Matrix<Id, Item>& matrix,
                 std::span<const Id> order, LabelFn&& label, ValueFn&& value) {
    const std::size_t n = order.size();

    std::vector<std::string> labels;
    labels.reserve(n);
    for (const Id& id : order) labels.push_back(label(id));

    std::vector<std::string> cells(n * n);
    std::size_t cellW = 1;  // 'x'
    for (std::size_t r = 0; r < n; ++r) {
        cellW = std::max(cellW, labels[r].size());
        for (std::size_t c = 0; c < n; ++c) {
            std::string& cell = cells[r * n + c];
            if (r == c)
                cell = "x";
            else if (const Item* item = matrix.find(order[r], order[c]))
                cell = value(*item);
            cellW = std::max(cellW, cell.size());
        }
    }

    const std::size_t rowLabelW =
        n ? std::ranges::max(labels, {}, &std::string::size).size() : 0;

    auto printTrimmed = [](std::string line) {
        line.resize(line.find_last_not_of(' ') + 1);
        std::println("{}", line);
    };

    std::println("=== {} ===", title);
    std::string header(rowLabelW, ' ');
    for (const std::string& l : labels)
        header += std::format(" {:>{}}", l, cellW);
    printTrimmed(std::move(header));

    for (std::size_t r = 0; r < n; ++r) {
        std::string line = std::format("{:<{}}", labels[r], rowLabelW);
        for (std::size_t c = 0; c < n; ++c)
            line += std::format(" {:>{}}", cells[r * n + c], cellW);
        printTrimmed(std::move(line));
    }
    std::println();
}

// Convenience overload formatting values with std::format's default "{}".
template <typename Id, typename Item, typename LabelFn>
void printMatrix(std::string_view title, const Matrix<Id, Item>& matrix,
                 std::span<const Id> order, LabelFn&& label) {
    printMatrix(title, matrix, order, std::forward<LabelFn>(label),
                [](const Item& v) { return std::format("{}", v); });
}

}  // namespace logs
