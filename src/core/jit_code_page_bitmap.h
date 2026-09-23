#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

// Tiny conservative bitmap used by dynarecs to remember physical pages that
// have ever contained translated guest code. Pages intentionally remain marked
// until clear(): stale positives only cost an uncommon invalidation check,
// while false negatives could miss self-modifying code.
//
// This is architecture-neutral on purpose. A PS1 backend can instantiate a
// 29-bit physical address space with 4 KiB pages; a future PS2 EE/IOP backend
// can reuse the same primitive with its own address width/page size.
template <unsigned AddressBits, unsigned PageShift>
class JitCodePageBitmap {
 public:
  static_assert(AddressBits > PageShift);
  static_assert(AddressBits <= 32u);
  static_assert(PageShift < 32u);

  using Word = std::uint64_t;

  static constexpr std::size_t kPageCount =
      std::size_t{1} << (AddressBits - PageShift);
  static constexpr std::size_t kBitsPerWord =
      std::numeric_limits<Word>::digits;
  static constexpr std::size_t kWordCount =
      (kPageCount + kBitsPerWord - 1u) / kBitsPerWord;

  void clear() noexcept { words_.fill(0); }

  void mark_page(std::uint32_t page) noexcept {
    const std::size_t p = static_cast<std::size_t>(page) & (kPageCount - 1u);
    words_[p / kBitsPerWord] |=
        Word{1} << static_cast<unsigned>(p % kBitsPerWord);
  }

  void mark_address(std::uint32_t address) noexcept {
    mark_page(address >> PageShift);
  }

  [[nodiscard]] bool test_page(std::uint32_t page) const noexcept {
    const std::size_t p = static_cast<std::size_t>(page) & (kPageCount - 1u);
    return (words_[p / kBitsPerWord] &
            (Word{1} << static_cast<unsigned>(p % kBitsPerWord))) != 0;
  }

  [[nodiscard]] bool test_address(std::uint32_t address) const noexcept {
    return test_page(address >> PageShift);
  }

  [[nodiscard]] bool any_page(std::uint32_t first_page,
                              std::uint32_t last_page) const noexcept {
    if (last_page < first_page) {
      return false;
    }

    const std::size_t first =
        static_cast<std::size_t>(first_page) & (kPageCount - 1u);
    const std::size_t last =
        static_cast<std::size_t>(last_page) & (kPageCount - 1u);
    if (last < first) {
      return false;
    }

    const std::size_t first_word = first / kBitsPerWord;
    const std::size_t last_word = last / kBitsPerWord;
    const unsigned first_bit = static_cast<unsigned>(first % kBitsPerWord);
    const unsigned last_bit = static_cast<unsigned>(last % kBitsPerWord);

    if (first_word == last_word) {
      const Word lower = ~Word{0} << first_bit;
      const Word upper =
          last_bit == kBitsPerWord - 1u
              ? ~Word{0}
              : ((Word{1} << (last_bit + 1u)) - 1u);
      return (words_[first_word] & lower & upper) != 0;
    }

    if ((words_[first_word] & (~Word{0} << first_bit)) != 0) {
      return true;
    }
    for (std::size_t word = first_word + 1u; word < last_word; ++word) {
      if (words_[word] != 0) {
        return true;
      }
    }

    const Word last_mask =
        last_bit == kBitsPerWord - 1u
            ? ~Word{0}
            : ((Word{1} << (last_bit + 1u)) - 1u);
    return (words_[last_word] & last_mask) != 0;
  }

  [[nodiscard]] const Word *data() const noexcept { return words_.data(); }
  [[nodiscard]] Word *data() noexcept { return words_.data(); }

 private:
  std::array<Word, kWordCount> words_{};
};
