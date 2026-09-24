#include "core/ee/ee_jit.h"
#include "core/ee/ee_cpu.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#if defined(_M_X64) || defined(__x86_64__)
#define VIBESTATION_EE_JIT_X64 1
#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/mman.h>
#endif
#endif

namespace ps2 {
namespace {

#if defined(VIBESTATION_EE_JIT_X64)
constexpr std::size_t kPageSize = 64u * 1024u;
constexpr std::size_t kMaxPages = 256u;
#ifdef _WIN32
constexpr u8 kArgumentRegister = 1u; // RCX
#else
constexpr u8 kArgumentRegister = 7u; // RDI
#endif

struct Emitter {
    std::vector<u8> bytes;

    void emit(u8 value) { bytes.push_back(value); }
    void emit32(u32 value) {
        for (u32 i = 0; i < 4; ++i) emit(static_cast<u8>(value >> (i * 8)));
    }
    void emit64(u64 value) {
        for (u32 i = 0; i < 8; ++i) emit(static_cast<u8>(value >> (i * 8)));
    }
    void memory(u8 rex, u8 opcode, u8 reg, u32 offset) {
        if (rex != 0) emit(rex);
        emit(opcode);
        emit(static_cast<u8>(0x80u | (reg << 3) | kArgumentRegister));
        emit32(offset);
    }
    void load_rax(u32 reg, bool word) {
        memory(word ? 0u : 0x48u, 0x8Bu, 0u,
               static_cast<u32>(offsetof(EeCpuState, gpr) +
                                reg * sizeof(EeGpr)));
    }
    void load_rdx(u32 reg, bool word) {
        memory(word ? 0u : 0x48u, 0x8Bu, 2u,
               static_cast<u32>(offsetof(EeCpuState, gpr) +
                                reg * sizeof(EeGpr)));
    }
    void store_rax(u32 reg) {
        memory(0x48u, 0x89u, 0u,
               static_cast<u32>(offsetof(EeCpuState, gpr) +
                                reg * sizeof(EeGpr)));
    }
    void store_gpr_imm64(u32 reg, u64 value) {
        emit(0x48u);
        emit(0xB8u);
        emit64(value);
        store_rax(reg);
    }
    void store_state_imm32(u32 offset, u32 value) {
        emit(0xC7u);
        emit(static_cast<u8>(0x80u | kArgumentRegister));
        emit32(offset);
        emit32(value);
    }
    void load_state_eax(u32 offset) {
        memory(0u, 0x8Bu, 0u, offset);
    }
    void store_state_eax(u32 offset) {
        memory(0u, 0x89u, 0u, offset);
    }
    std::size_t jcc32(u8 condition) {
        emit(0x0Fu);
        emit(condition);
        const std::size_t displacement = bytes.size();
        emit32(0u);
        return displacement;
    }
    void patch_rel32(std::size_t displacement, std::size_t target) {
        const s64 rel = static_cast<s64>(target) -
            static_cast<s64>(displacement + 4u);
        const u32 encoded = static_cast<u32>(static_cast<s32>(rel));
        for (u32 i = 0; i < 4u; ++i) {
            bytes[displacement + i] =
                static_cast<u8>(encoded >> (i * 8u));
        }
    }
    void sign_extend_word() { emit(0x48u); emit(0x98u); } // CDQE
};

void* allocate_page() {
#ifdef _WIN32
    return VirtualAlloc(nullptr, kPageSize, MEM_COMMIT | MEM_RESERVE,
                        PAGE_READWRITE);
#else
    void* page = mmap(nullptr, kPageSize, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    return page == MAP_FAILED ? nullptr : page;
#endif
}

bool protect_page(void* page, bool executable) {
#ifdef _WIN32
    DWORD old_protection = 0;
    return VirtualProtect(page, kPageSize,
                          executable ? PAGE_EXECUTE_READ : PAGE_READWRITE,
                          &old_protection) != 0;
#else
    return mprotect(page, kPageSize,
                    executable ? PROT_READ | PROT_EXEC
                               : PROT_READ | PROT_WRITE) == 0;
#endif
}

void release_page(void* page) {
#ifdef _WIN32
    VirtualFree(page, 0, MEM_RELEASE);
#else
    munmap(page, kPageSize);
#endif
}

void flush_code(void* code, std::size_t size) {
#ifdef _WIN32
    FlushInstructionCache(GetCurrentProcess(), code, size);
#else
    auto* first = static_cast<char*>(code);
    __builtin___clear_cache(first, first + size);
#endif
}

bool emit_instruction_body(u32 instruction, Emitter& out) {
    const u32 opcode = instruction >> 26;
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 rd = (instruction >> 11) & 31u;
    const u32 sa = (instruction >> 6) & 31u;
    const u32 immediate = instruction & 0xFFFFu;
    u32 destination = rt;

    if (opcode == 0u) {
        const u32 funct = instruction & 63u;
        destination = rd;
        switch (funct) {
        case 0x00u: // SLL / NOP
        case 0x02u: // SRL
        case 0x03u: // SRA
            if (rs != 0u) return false;
            if (destination != 0u) {
                out.load_rax(rt, true);
                out.emit(0xC1u);
                out.emit(funct == 0x00u ? 0xE0u :
                         funct == 0x02u ? 0xE8u : 0xF8u);
                out.emit(static_cast<u8>(sa));
                out.sign_extend_word();
            }
            break;
        case 0x21u: // ADDU
        case 0x23u: // SUBU
        case 0x24u: // AND
        case 0x25u: // OR
        case 0x26u: // XOR
        case 0x2Au: // SLT
        case 0x2Bu: // SLTU
        case 0x2Du: // DADDU
            if (sa != 0u) return false;
            if (destination != 0u) {
                const bool word = funct == 0x21u || funct == 0x23u;
                out.load_rax(rs, word);
                out.load_rdx(rt, word);
                if (funct == 0x2Au || funct == 0x2Bu) {
                    out.emit(0x48u);
                    out.emit(0x39u);
                    out.emit(0xD0u); // CMP RAX,RDX
                    out.emit(0x0Fu);
                    out.emit(funct == 0x2Au ? 0x9Cu : 0x92u);
                    out.emit(0xC0u); // SETL / SETB AL
                    out.emit(0x0Fu);
                    out.emit(0xB6u);
                    out.emit(0xC0u); // MOVZX EAX,AL
                } else {
                    if (!word) out.emit(0x48u);
                    out.emit(funct == 0x23u ? 0x29u :
                             funct == 0x24u ? 0x21u :
                             funct == 0x25u ? 0x09u :
                             funct == 0x26u ? 0x31u : 0x01u);
                    out.emit(0xD0u); // operation RAX, RDX
                    if (word) out.sign_extend_word();
                }
            }
            break;
        default:
            return false;
        }
    } else {
        switch (opcode) {
        case 0x09u: // ADDIU
            if (destination != 0u) {
                out.load_rax(rs, true);
                out.emit(0x05u); // ADD EAX, imm32
                out.emit32(static_cast<u32>(static_cast<s32>(
                    static_cast<s16>(immediate))));
                out.sign_extend_word();
            }
            break;
        case 0x0Au: // SLTI
        case 0x0Bu: // SLTIU
            if (destination != 0u) {
                out.load_rax(rs, false);
                out.emit(0x48u);
                out.emit(0x3Du); // CMP RAX, sign-extended imm32
                out.emit32(static_cast<u32>(static_cast<s32>(
                    static_cast<s16>(immediate))));
                out.emit(0x0Fu);
                out.emit(opcode == 0x0Au ? 0x9Cu : 0x92u); // SETL / SETB
                out.emit(0xC0u);
                out.emit(0x0Fu);
                out.emit(0xB6u);
                out.emit(0xC0u); // MOVZX EAX,AL
            }
            break;
        case 0x0Cu: // ANDI
        case 0x0Du: // ORI
        case 0x0Eu: // XORI
            if (destination != 0u) {
                out.load_rax(rs, false);
                out.emit(0x48u);
                out.emit(opcode == 0x0Cu ? 0x25u :
                         opcode == 0x0Du ? 0x0Du : 0x35u);
                out.emit32(immediate);
            }
            break;
        case 0x0Fu: // LUI
            if (destination != 0u) {
                out.emit(0xB8u); // MOV EAX, imm32
                out.emit32(immediate << 16);
                out.sign_extend_word();
            }
            break;
        case 0x19u: // DADDIU
            if (destination != 0u) {
                out.load_rax(rs, false);
                out.emit(0x48u);
                out.emit(0x05u); // ADD RAX, sign-extended imm32
                out.emit32(static_cast<u32>(static_cast<s32>(
                    static_cast<s16>(immediate))));
            }
            break;
        default:
            return false;
        }
    }

    if (destination != 0u) out.store_rax(destination);
    return true;
}

bool emit_instruction(u32 instruction, Emitter& out) {
    if (!emit_instruction_body(instruction, out)) return false;
    out.emit(0xC3u); // RET
    return true;
}

bool emit_branch_and_delay(
    u32 branch_pc,
    u32 branch_instruction,
    u32 delay_instruction,
    Emitter& out) {
    const std::size_t before = out.bytes.size();
    const u32 opcode = branch_instruction >> 26;
    const u32 rs = (branch_instruction >> 21) & 31u;
    const u32 rt = (branch_instruction >> 16) & 31u;
    const s16 imm = static_cast<s16>(branch_instruction & 0xFFFFu);
    const u32 pc_offset = static_cast<u32>(offsetof(EeCpuState, pc));
    const u32 next_pc_offset =
        static_cast<u32>(offsetof(EeCpuState, next_pc));
    const u32 fallthrough = branch_pc + 8u;
    const u32 target = branch_pc + 4u +
        static_cast<u32>(static_cast<s32>(imm) * 4);

    if (opcode == 0x00u) {
        const u32 funct = branch_instruction & 63u;
        if (funct == 0x08u || funct == 0x09u) { // JR / JALR
            const u32 rd = (branch_instruction >> 11) & 31u;
            out.load_rax(rs, true);
            out.store_state_eax(pc_offset);
            if (funct == 0x09u && rd != 0u) {
                const u64 link = static_cast<u64>(static_cast<s64>(
                    static_cast<s32>(branch_pc + 8u)));
                out.store_gpr_imm64(rd, link);
            }
            if (!emit_instruction_body(delay_instruction, out)) {
                out.bytes.resize(before);
                return false;
            }
            out.load_state_eax(pc_offset);
            out.emit(0x05u);
            out.emit32(4u);
            out.store_state_eax(next_pc_offset);
            return true;
        }
    }

    switch (opcode) {
    case 0x02u: // J
    case 0x03u: { // JAL
        const u32 jump_target =
            ((branch_pc + 4u) & 0xF0000000u) |
            ((branch_instruction & 0x03FFFFFFu) << 2);
        if (opcode == 0x03u) {
            const u64 link = static_cast<u64>(static_cast<s64>(
                static_cast<s32>(branch_pc + 8u)));
            out.store_gpr_imm64(31u, link);
        }
        out.store_state_imm32(pc_offset, jump_target);
        break;
    }
    case 0x04u: // BEQ
    case 0x05u: { // BNE
        out.store_state_imm32(pc_offset, fallthrough);
        out.load_rax(rs, false);
        out.load_rdx(rt, false);
        out.emit(0x48u);
        out.emit(0x39u);
        out.emit(0xD0u); // CMP RAX,RDX
        const std::size_t skip_target =
            out.jcc32(opcode == 0x04u ? 0x85u : 0x84u);
        out.store_state_imm32(pc_offset, target);
        out.patch_rel32(skip_target, out.bytes.size());
        break;
    }
    case 0x06u: // BLEZ
    case 0x07u: { // BGTZ
        out.store_state_imm32(pc_offset, fallthrough);
        out.load_rax(rs, false);
        out.emit(0x48u);
        out.emit(0x85u);
        out.emit(0xC0u); // TEST RAX,RAX
        const std::size_t skip_target =
            out.jcc32(opcode == 0x06u ? 0x8Fu : 0x8Eu);
        out.store_state_imm32(pc_offset, target);
        out.patch_rel32(skip_target, out.bytes.size());
        break;
    }
    default:
        out.bytes.resize(before);
        return false;
    }

    if (!emit_instruction_body(delay_instruction, out)) {
        out.bytes.resize(before);
        return false;
    }

    out.load_state_eax(pc_offset);
    out.emit(0x05u); // ADD EAX, imm32
    out.emit32(4u);
    out.store_state_eax(next_pc_offset);
    return true;
}

bool emit_block(
    u32 pc,
    const u32* instructions,
    u32 instruction_count,
    Emitter& out,
    u32& compiled_instructions,
    bool& control_flow) {
    compiled_instructions = 0;
    control_flow = false;
    for (u32 i = 0; i < instruction_count; ++i) {
        const std::size_t before = out.bytes.size();
        if (emit_instruction_body(instructions[i], out)) {
            ++compiled_instructions;
            continue;
        }

        out.bytes.resize(before);
        if (i + 1u < instruction_count &&
            emit_branch_and_delay(
                pc + i * 4u,
                instructions[i],
                instructions[i + 1u],
                out)) {
            compiled_instructions += 2u;
            control_flow = true;
        }
        break;
    }
    if (compiled_instructions == 0u) return false;
    out.emit(0xC3u); // RET
    return true;
}
#endif

} // namespace

EeJit::~EeJit() { clear(); }

void EeJit::clear() {
#if defined(VIBESTATION_EE_JIT_X64)
    for (const Page& page : pages_) release_page(page.address);
#endif
    pages_.clear();
    entries_ = {};
    std::fill(block_entries_.begin(), block_entries_.end(), BlockEntry{});
    compiled_count_ = 0;
    executed_count_ = 0;
    block_compiled_count_ = 0;
    block_executed_count_ = 0;
    block_instruction_count_ = 0;
}

EeJit::Function EeJit::compile(u32 instruction) {
#if defined(VIBESTATION_EE_JIT_X64)
    Emitter emitter;
    if (!emit_instruction(instruction, emitter)) return nullptr;
    if (pages_.empty() ||
        pages_.back().used + emitter.bytes.size() > kPageSize) {
        if (pages_.size() >= kMaxPages) clear();
        void* address = allocate_page();
        if (address == nullptr) return nullptr;
        pages_.push_back(Page{address, 0});
    }
    Page& page = pages_.back();
    if (page.used != 0 && !protect_page(page.address, false)) return nullptr;
    auto* code = static_cast<u8*>(page.address) + page.used;
    std::memcpy(code, emitter.bytes.data(), emitter.bytes.size());
    if (!protect_page(page.address, true)) return nullptr;
    flush_code(code, emitter.bytes.size());
    page.used += emitter.bytes.size();
    ++compiled_count_;
    return reinterpret_cast<Function>(code);
#else
    (void)instruction;
    return nullptr;
#endif
}

EeJit::Function EeJit::compile_block(
    u32 pc,
    const u32* instructions,
    u32 instruction_count,
    u32& compiled_instructions,
    bool& control_flow) {
#if defined(VIBESTATION_EE_JIT_X64)
    Emitter emitter;
    if (!emit_block(
            pc,
            instructions,
            instruction_count,
            emitter,
            compiled_instructions,
            control_flow)) {
        return nullptr;
    }

    if (pages_.empty() ||
        pages_.back().used + emitter.bytes.size() > kPageSize) {
        if (pages_.size() >= kMaxPages) clear();
        void* address = allocate_page();
        if (address == nullptr) return nullptr;
        pages_.push_back(Page{address, 0});
    }

    Page& page = pages_.back();
    if (page.used != 0 && !protect_page(page.address, false)) return nullptr;
    auto* code = static_cast<u8*>(page.address) + page.used;
    std::memcpy(code, emitter.bytes.data(), emitter.bytes.size());
    if (!protect_page(page.address, true)) return nullptr;
    flush_code(code, emitter.bytes.size());
    page.used += emitter.bytes.size();
    ++block_compiled_count_;
    return reinterpret_cast<Function>(code);
#else
    (void)pc;
    (void)instructions;
    (void)instruction_count;
    compiled_instructions = 0;
    control_flow = false;
    return nullptr;
#endif
}

u32 EeJit::execute_block(
    EeCpuState& state,
    u32 pc,
    u32 page_generation,
    const u32* instructions,
    u32 instruction_count,
    u32 maximum_instructions,
    bool& control_flow) {
#if defined(VIBESTATION_EE_JIT_X64)
    if (instructions == nullptr ||
        instruction_count == 0u ||
        maximum_instructions == 0u) {
        control_flow = false;
        return 0;
    }

    const u32 hash =
        (pc >> 2) * 2654435761u ^
        page_generation * 2246822519u;
    const std::size_t index =
        (static_cast<std::size_t>(hash) *
         block_entries_.size()) >> 32;
    BlockEntry& entry = block_entries_[index];

    if (!entry.known ||
        entry.pc != pc ||
        entry.page_generation != page_generation) {
        u32 compiled_instructions = 0;
        bool compiled_control_flow = false;
        Function function = compile_block(
            pc,
            instructions,
            instruction_count,
            compiled_instructions,
            compiled_control_flow);
        entry.pc = pc;
        entry.page_generation = page_generation;
        entry.instruction_count =
            static_cast<u8>(compiled_instructions);
        entry.function = function;
        entry.control_flow = compiled_control_flow;
        entry.known = true;
    }

    if (entry.function == nullptr ||
        entry.instruction_count == 0u ||
        entry.instruction_count > maximum_instructions) {
        control_flow = false;
        return 0;
    }

    entry.function(&state);
    control_flow = entry.control_flow;
    ++block_executed_count_;
    block_instruction_count_ += entry.instruction_count;
    return entry.instruction_count;
#else
    (void)state;
    (void)pc;
    (void)page_generation;
    (void)instructions;
    (void)instruction_count;
    (void)maximum_instructions;
    control_flow = false;
    return 0;
#endif
}

bool EeJit::execute(EeCpuState& state, u32 instruction) {
#if defined(VIBESTATION_EE_JIT_X64)
    const u32 hash = instruction * 2654435761u;
    const std::size_t index = hash >> 20;
    Entry& entry = entries_[index];
    if (!entry.known || entry.instruction != instruction) {
        entry.instruction = instruction;
        entry.function = compile(instruction);
        entry.known = true;
    }
    if (entry.function == nullptr) return false;
    entry.function(&state);
    ++executed_count_;
    return true;
#else
    (void)state;
    (void)instruction;
    return false;
#endif
}

} // namespace ps2
