#pragma once
#include <expected>
#include <type_traits>

#include "bus/network_event_bus.hpp"
#include "command/transaction.hpp"
#include "command_stack.hpp"
#include "document.hpp"
#include "editor.hpp"

template <class T>
struct is_expected : std::false_type {};
template <class T, class E>
struct is_expected<std::expected<T, E>> : std::true_type {};
template <class T>
inline constexpr bool is_expected_v = is_expected<T>::value;

class Project {
   public:
    explicit Project(Document doc)
        : doc_(std::move(doc)), editor_(doc_, bus_) {}

    const Document& document() { return doc_; }
    const NetworkEventBus& bus() const { return bus_; }
    NetworkEventBus& bus() { return bus_; }
    CommandStack& commands() { return stack_; }

    // Runs body(tx, editor) as one undo entry. A returned std::expected error
    // rolls the transaction back (via ~Transaction) and is passed through.
    template <class Body>
    auto run(Body&& body) {
        Transaction tx;
        editor_.registerSolves(tx);
        using R = std::invoke_result_t<Body&, Transaction&, Editor&>;

        if constexpr (std::is_void_v<R>) {
            std::forward<Body>(body)(tx, editor_);
            stack_.push(tx.commit());  // commit runs solves then events
        } else {
            R r = std::forward<Body>(body)(tx, editor_);
            if constexpr (is_expected_v<R>)
                if (!r) return r;  // roll back
            stack_.push(tx.commit());
            return r;
        }
    }

   private:
    Document doc_;
    NetworkEventBus bus_;
    Editor editor_;
    CommandStack stack_;
};