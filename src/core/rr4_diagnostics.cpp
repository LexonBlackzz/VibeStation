#include "rr4_diagnostics.h"
#include "system.h"

namespace {

bool is_plausible_exec_addr_rr4(u32 addr) {
  if (addr >= 0x00000000u && addr < 0x00800000u) return true;
  if (addr >= 0x80000000u && addr < 0x80800000u) return true;
  if (addr >= 0xA0000000u && addr < 0xA0800000u) return true;
  if (addr >= 0xBFC00000u && addr < 0xBFC80000u) return true;
  return false;
}

void log_stack_window_rr4(System *sys, const char *label, u32 sp) {
  if (sys == nullptr || (sp & 3u) != 0u || !is_plausible_exec_addr_rr4(sp)) {
    return;
  }
  LOG_WARN(
      "%s sp=0x%08X [sp+10]=0x%08X [sp+14]=0x%08X [sp+18]=0x%08X [sp+1C]=0x%08X",
      label, sp, sys->read32(sp + 0x10u), sys->read32(sp + 0x14u),
      sys->read32(sp + 0x18u), sys->read32(sp + 0x1Cu));
}

} // namespace

namespace rr4_diag {

namespace {
u32 instr_rs(u32 i) { return (i >> 21) & 0x1F; }
u32 instr_rt(u32 i) { return (i >> 16) & 0x1F; }
s32 instr_simm(u32 i) {
  return sign_extend_16(static_cast<u16>(i & 0xFFFF));
}
bool is_main_ram_addr_rr4(u32 addr) {
  return (addr & 0x1FFFFFFFu) < 0x00200000u;
}
} // namespace

void on_load32(Rr4DiagState& st, u32 pc, u32 addr, u32 value,
               const u32* regs, u64 cycles, System& sys) {
  if (g_log_fmv_diagnostics && pc == 0x00000DE8u &&
      (addr & 0x1FFFFFFFu) == 0x00000018u && value == 0x00000001u) {
    if (st.slot18_sentinel < 32u) {
      ++st.slot18_sentinel;
      LOG_WARN(
          "CPU: rr4 slot18 sentinel addr=0x%08X val=0x%08X pass-through "
          "pc=0x%08X ra=0x%08X s3=0x%08X cyc=%llu",
          addr, value, pc, regs[31], regs[19],
          static_cast<unsigned long long>(cycles));
    }
  }
  if (g_log_fmv_diagnostics && pc == 0x00000E28u &&
      (addr & 0x1FFFFFFFu) == 0x000A6518u && value == 0x00000001u) {
    if (st.next_sentinel < 32u) {
      ++st.next_sentinel;
      LOG_WARN(
          "CPU: rr4 next sentinel addr=0x%08X val=0x%08X pass-through "
          "pc=0x%08X ra=0x%08X s1=0x%08X s3=0x%08X cyc=%llu",
          addr, value, pc, regs[31], regs[17], regs[19],
          static_cast<unsigned long long>(cycles));
    }
  }
  if (g_log_fmv_diagnostics && pc == 0x8008B8FCu &&
      (addr & 0x1FFFFFFFu) == 0x000A6518u && value == 0x00000001u) {
    if (st.head_sentinel < 32u) {
      ++st.head_sentinel;
      LOG_WARN(
          "CPU: rr4 head sentinel addr=0x%08X val=0x%08X pass-through "
          "pc=0x%08X ra=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X cyc=%llu",
          addr, value, pc, regs[31], regs[17], regs[18], regs[19],
          static_cast<unsigned long long>(cycles));
    }
  }
}

void on_exception(Rr4DiagState& st, u32 pc, u64 cycles,
                  u32 cop0_epc, const u32* regs, System& sys) {
  if (g_log_fmv_diagnostics &&
      (pc >= 0x800970B0u && pc <= 0x80097110u) && cycles >= 390000000ull) {
    if (st.syscall_logs < 128u) {
      ++st.syscall_logs;
      LOG_WARN(
          "CPU: rr4-syscall pc=0x%08X epc=0x%08X cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "t0=0x%08X t1=0x%08X t2=0x%08X ra=0x%08X sp=0x%08X "
          "lowB0=0x%08X lowB4=0x%08X lowB8=0x%08X lowBC=0x%08X "
          "lowC0=0x%08X",
          pc, cop0_epc, static_cast<unsigned long long>(cycles),
          regs[2], regs[3], regs[4], regs[5], regs[6], regs[7], regs[8],
          regs[9], regs[10], regs[31], regs[29], sys.read32(0x000000B0u),
          sys.read32(0x000000B4u), sys.read32(0x000000B8u),
          sys.read32(0x000000BCu), sys.read32(0x000000C0u));
    }
  }
}

void on_step(Rr4DiagState& st, u32 pc, u32 instruction, const u32* regs,
             u64 cycles, bool cpu_diag, u32 cop0_sr, u32 cop0_cause,
             bool irq_pending, System& sys) {
  const u32 prev = st.prev_pc_for_diag;

  if (g_log_fmv_diagnostics && pc >= 0x80090540u &&
      pc <= 0x80090610u) {
    if (st.dma_setup < 512u) {
      ++st.dma_setup;
      LOG_INFO(
          "CPU: RR4 DMA setup pc=0x%08X instr=0x%08X cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X t4=0x%08X t5=0x%08X "
          "t6=0x%08X t7=0x%08X t8=0x%08X t9=0x%08X sp=0x%08X ra=0x%08X "
          "h0=0x%08X h1=0x%08X h2=0x%08X h3=0x%08X",
          pc, instruction, static_cast<unsigned long long>(cycles),
          regs[2], regs[3], regs[4], regs[5], regs[6], regs[7], regs[8],
          regs[9], regs[10], regs[11], regs[12], regs[13], regs[14],
          regs[15], regs[24], regs[25], regs[29], regs[31],
          sys.read32(0x00141BF4u), sys.read32(0x00141BF8u),
          sys.read32(0x00141BFCu), sys.read32(0x00141C00u));
    }
  }

  if (g_log_fmv_diagnostics && pc >= 0x8008FC80u &&
      pc <= 0x8008FF40u) {
    if (st.sector_parse < 3072u) {
      ++st.sector_parse;
      LOG_INFO(
          "CPU: RR4 sector parse pc=0x%08X instr=0x%08X cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X t4=0x%08X t5=0x%08X "
          "t6=0x%08X t7=0x%08X s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X "
          "s4=0x%08X sp=0x%08X ra=0x%08X buf0=0x%08X buf1=0x%08X "
          "buf2=0x%08X buf3=0x%08X cdchcr=0x%08X dicr=0x%08X "
          "st18=0x%08X st1c=0x%08X st24=0x%08X st28=0x%08X "
          "st2c=0x%08X st30=0x%08X st34=0x%08X st38=0x%08X "
          "st3c=0x%08X st40=0x%08X st78=0x%08X rp=0x%08X wp=0x%08X",
          pc, instruction, static_cast<unsigned long long>(cycles),
          regs[2], regs[3], regs[4], regs[5], regs[6], regs[7], regs[8],
          regs[9], regs[10], regs[11], regs[12], regs[13], regs[14],
          regs[15], regs[16], regs[17], regs[18], regs[19], regs[20],
          regs[29], regs[31], sys.read32(0x00141BF4u),
          sys.read32(0x00141BF8u), sys.read32(0x00141BFCu),
          sys.read32(0x00141C00u), sys.read32(0x1F8010B8u),
          sys.read32(0x1F8010F4u), sys.read32(0x00110418u),
          sys.read32(0x0011041Cu), sys.read32(0x00110424u),
          sys.read32(0x00110428u), sys.read32(0x0011042Cu),
          sys.read32(0x00110430u), sys.read32(0x00110434u),
          sys.read32(0x00110438u), sys.read32(0x0011043Cu),
          sys.read32(0x00110440u), sys.read32(0x00110478u),
          sys.read32(0x00117788u), sys.read32(0x00127788u));
    }
  }

  if (g_log_fmv_diagnostics && pc >= 0x8008FD68u &&
      pc <= 0x8008FE90u) {
    if (st.payload_decision < 1024u) {
      ++st.payload_decision;
      LOG_INFO(
          "CPU: RR4 payload decision pc=0x%08X instr=0x%08X cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X t4=0x%08X "
          "s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X sp=0x%08X ra=0x%08X "
          "h0=0x%08X h1=0x%08X h2=0x%08X h3=0x%08X "
          "st18=0x%08X st24=0x%08X st28=0x%08X st2c=0x%08X "
          "st30=0x%08X st34=0x%08X st38=0x%08X st3c=0x%08X "
          "st40=0x%08X st78=0x%08X cdchcr=0x%08X dicr=0x%08X",
          pc, instruction, static_cast<unsigned long long>(cycles),
          regs[2], regs[3], regs[4], regs[5], regs[6], regs[7], regs[8],
          regs[9], regs[10], regs[11], regs[12], regs[16], regs[17],
          regs[18], regs[19], regs[29], regs[31], sys.read32(0x00141BF4u),
          sys.read32(0x00141BF8u), sys.read32(0x00141BFCu),
          sys.read32(0x00141C00u), sys.read32(0x00110418u),
          sys.read32(0x00110424u), sys.read32(0x00110428u),
          sys.read32(0x0011042Cu), sys.read32(0x00110430u),
          sys.read32(0x00110434u), sys.read32(0x00110438u),
          sys.read32(0x0011043Cu), sys.read32(0x00110440u),
          sys.read32(0x00110478u), sys.read32(0x1F8010B8u),
          sys.read32(0x1F8010F4u));
    }
  }

  if (g_log_fmv_diagnostics && pc >= 0x8008F9D0u &&
      pc <= 0x8008FA40u) {
    if (st.post_parse < 768u) {
      ++st.post_parse;
      LOG_INFO(
          "CPU: RR4 post-parse pc=0x%08X instr=0x%08X cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X s0=0x%08X s1=0x%08X "
          "s2=0x%08X s3=0x%08X sp=0x%08X ra=0x%08X "
          "st0=0x%08X st1=0x%08X st2=0x%08X st3=0x%08X st4=0x%08X "
          "rp=0x%08X wp=0x%08X",
          pc, instruction, static_cast<unsigned long long>(cycles),
          regs[2], regs[3], regs[4], regs[5], regs[6], regs[7], regs[8],
          regs[9], regs[10], regs[11], regs[16], regs[17], regs[18],
          regs[19], regs[29], regs[31], sys.read32(0x00110400u),
          sys.read32(0x00110404u), sys.read32(0x00110424u),
          sys.read32(0x00110428u), sys.read32(0x00110438u),
          sys.read32(0x00117788u), sys.read32(0x00127788u));
    }
  }

  if (g_log_fmv_diagnostics && pc >= 0x00000C80u &&
      pc <= 0x00000E40u) {
    const bool came_from_outside =
        !(prev >= 0x00000C80u && prev <= 0x00000E40u);
    if (came_from_outside) {
      st.low_runtime_active = false;
    }
    if (!st.low_runtime_full_dumped) {
      st.low_runtime_full_dumped = true;
      for (u32 base = 0x00000C80u; base <= 0x00000E30u; base += 0x10u) {
        LOG_WARN(
            "CPU: low-runtime full %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
            base + 0x00u, sys.read32(base + 0x00u), base + 0x04u,
            sys.read32(base + 0x04u), base + 0x08u, sys.read32(base + 0x08u),
            base + 0x0Cu, sys.read32(base + 0x0Cu));
      }
    }
    if (!st.low_runtime_active && st.low_runtime_log_count < 24u) {
      st.low_runtime_active = true;
      ++st.low_runtime_log_count;
      const u32 phys_pc = pc & 0x1FFFFFFFu;
      LOG_WARN(
          "CPU: entered low-runtime prev_pc=0x%08X pc=0x%08X instr=0x%08X "
          "sp=0x%08X ra=0x%08X v0=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X "
          "a3=0x%08X",
          prev, pc, instruction, regs[29], regs[31], regs[2], regs[4],
          regs[5], regs[6], regs[7]);
      LOG_WARN(
          "CPU: low-runtime s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X "
          "s4=0x%08X s5=0x%08X s6=0x%08X s7=0x%08X",
          regs[16], regs[17], regs[18], regs[19], regs[20], regs[21],
          regs[22], regs[23]);
      LOG_WARN(
          "CPU: low-runtime code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          phys_pc - 0x08u, sys.read32(phys_pc - 0x08u), phys_pc - 0x04u,
          sys.read32(phys_pc - 0x04u), phys_pc + 0x00u,
          sys.read32(phys_pc + 0x00u), phys_pc + 0x04u,
          sys.read32(phys_pc + 0x04u), phys_pc + 0x08u,
          sys.read32(phys_pc + 0x08u));
      log_stack_window_rr4(&sys, "CPU: low-runtime frame", regs[29]);
    }
  }

  if (g_log_fmv_diagnostics && pc >= 0x00000DE8u &&
      pc <= 0x00000E3Cu && cycles >= 780000000ull) {
    if (st.late_low_loop < 160u) {
      ++st.late_low_loop;
      const u32 s6 = regs[22];
      const bool s6_ram = is_main_ram_addr_rr4(s6);
      const u32 next0 = s6_ram ? sys.read32(s6 + 0x00u) : 0xFFFFFFFFu;
      const u32 next4 = s6_ram ? sys.read32(s6 + 0x04u) : 0xFFFFFFFFu;
      const u32 next8 = s6_ram ? sys.read32(s6 + 0x08u) : 0xFFFFFFFFu;
      const u32 low10 = sys.read32(0x00000010u);
      const u32 low18 = sys.read32(0x00000018u);
      const u32 bridge0 = sys.read32(0x800A6540u);
      const u32 bridge4 = sys.read32(0x800A6544u);
      const u32 bridge8 = sys.read32(0x800A6548u);
      LOG_WARN(
          "CPU: late low-loop prev=0x%08X pc=0x%08X instr=0x%08X cyc=%llu "
          "s6=0x%08X [0]=0x%08X [4]=0x%08X [8]=0x%08X "
          "s0=0x%08X s1=0x%08X s3=0x%08X s4=0x%08X "
          "v0=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "low[10]=0x%08X low[18]=0x%08X "
          "bridge=[0x%08X,0x%08X,0x%08X]",
          prev, pc, instruction, static_cast<unsigned long long>(cycles),
          s6, next0, next4, next8, regs[16], regs[17], regs[19], regs[20],
          regs[2], regs[4], regs[5], regs[6], regs[7], low10, low18,
          bridge0, bridge4, bridge8);
    }
  }

  if (g_log_fmv_diagnostics && pc >= 0x80082E80u &&
      pc <= 0x80082F10u && cycles >= 780000000ull) {
    if (st.callback2_exec < 192u) {
      ++st.callback2_exec;
      LOG_WARN(
          "CPU: rr4-cb exec prev=0x%08X pc=0x%08X instr=0x%08X cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X s6=0x%08X sp=0x%08X "
          "ra=0x%08X "
          "low[10]=0x%08X low[18]=0x%08X "
          "node=[0x%08X,0x%08X,0x%08X] bridge=[0x%08X,0x%08X,0x%08X]",
          prev, pc, instruction,
          static_cast<unsigned long long>(cycles), regs[2], regs[3], regs[4],
          regs[5], regs[6], regs[7], regs[16], regs[17], regs[18], regs[19],
          regs[22], regs[29], regs[31], sys.read32(0x00000010u),
          sys.read32(0x00000018u), sys.read32(0x800A6518u),
          sys.read32(0x800A651Cu), sys.read32(0x800A6520u),
          sys.read32(0x800A6540u), sys.read32(0x800A6544u),
          sys.read32(0x800A6548u));
    }
  }

  if (g_log_fmv_diagnostics && pc >= 0x00000EA0u &&
      pc <= 0x00000EE0u && cycles >= 780000000ull) {
    if (st.late_low_dispatcher < 128u) {
      ++st.late_low_dispatcher;
      const u32 phys_pc = pc & 0x1FFFFFFFu;
      LOG_WARN(
          "CPU: late low-dispatcher prev=0x%08X pc=0x%08X instr=0x%08X "
          "cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X s6=0x%08X sp=0x%08X "
          "ra=0x%08X "
          "low[10]=0x%08X low[18]=0x%08X",
          prev, pc, instruction,
          static_cast<unsigned long long>(cycles), regs[2], regs[3], regs[4],
          regs[5], regs[6], regs[7], regs[16], regs[17], regs[18], regs[19],
          regs[22], regs[29], regs[31], sys.read32(0x10u),
          sys.read32(0x18u));
      LOG_WARN(
          "CPU: late low-dispatcher code %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X %08X=%08X",
          phys_pc - 0x08u, sys.read32(phys_pc - 0x08u), phys_pc - 0x04u,
          sys.read32(phys_pc - 0x04u), phys_pc + 0x00u,
          sys.read32(phys_pc + 0x00u), phys_pc + 0x04u,
          sys.read32(phys_pc + 0x04u), phys_pc + 0x08u,
          sys.read32(phys_pc + 0x08u));
    }
  }

  if (g_log_fmv_diagnostics && pc >= 0x00000E00u &&
      pc <= 0x00000E10u && cycles >= 780000000ull) {
    if (st.late_low_callsite < 128u) {
      ++st.late_low_callsite;
      LOG_WARN(
          "CPU: late low-callsite prev=0x%08X pc=0x%08X instr=0x%08X "
          "cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X s6=0x%08X ra=0x%08X",
          prev, pc, instruction,
          static_cast<unsigned long long>(cycles), regs[2], regs[3], regs[4],
          regs[5], regs[6], regs[7], regs[16], regs[17], regs[18], regs[19],
          regs[22], regs[31]);
    }
  }

  if (g_log_fmv_diagnostics && pc >= 0x00000C90u &&
      pc <= 0x00000CA4u && cycles >= 900000000ull) {
    if (st.low_k0_chain < 192u) {
      ++st.low_k0_chain;
      const u32 phys_pc = pc & 0x1FFFFFFFu;
      LOG_WARN(
          "CPU: low-k0 chain prev=0x%08X pc=0x%08X instr=0x%08X cyc=%llu "
          "k0=0x%08X k1=0x%08X v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X "
          "a2=0x%08X a3=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X sp=0x%08X "
          "ra=0x%08X",
          prev, pc, instruction,
          static_cast<unsigned long long>(cycles), regs[26], regs[27],
          regs[2], regs[3], regs[4], regs[5], regs[6], regs[7], regs[17],
          regs[18], regs[19], regs[29], regs[31]);
      LOG_WARN(
          "CPU: low-k0 mem %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x00000100u, sys.read32(0x00000100u), 0x00000108u,
          sys.read32(0x00000108u), 0x00000110u, sys.read32(0x00000110u),
          0x00000118u, sys.read32(0x00000118u), 0x00000120u,
          sys.read32(0x00000120u));
      LOG_WARN(
          "CPU: low-k0 code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          phys_pc - 0x08u, sys.read32(phys_pc - 0x08u), phys_pc - 0x04u,
          sys.read32(phys_pc - 0x04u), phys_pc + 0x00u,
          sys.read32(phys_pc + 0x00u), phys_pc + 0x04u,
          sys.read32(phys_pc + 0x04u), phys_pc + 0x08u,
          sys.read32(phys_pc + 0x08u));
    }
  }

  if (g_log_fmv_diagnostics && pc >= 0x00000CA0u &&
      pc <= 0x00000CB8u && cycles >= 800000000ull &&
      sys.read32(0x10u) == 0x800A6518u) {
    if (st.low_dispatch < 96u) {
      ++st.low_dispatch;
      const u32 phys_pc = pc & 0x1FFFFFFFu;
      LOG_WARN(
          "CPU: low-dispatch prev=0x%08X pc=0x%08X instr=0x%08X cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X s4=0x%08X s6=0x%08X "
          "sp=0x%08X ra=0x%08X low[10]=0x%08X low[18]=0x%08X",
          prev, pc, instruction,
          static_cast<unsigned long long>(cycles), regs[2], regs[3], regs[4],
          regs[5], regs[6], regs[7], regs[16], regs[17], regs[18], regs[19],
          regs[20], regs[22], regs[29], regs[31], sys.read32(0x10u),
          sys.read32(0x18u));
      LOG_WARN(
          "CPU: low-dispatch code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          phys_pc - 0x08u, sys.read32(phys_pc - 0x08u), phys_pc - 0x04u,
          sys.read32(phys_pc - 0x04u), phys_pc + 0x00u,
          sys.read32(phys_pc + 0x00u), phys_pc + 0x04u,
          sys.read32(phys_pc + 0x04u), phys_pc + 0x08u,
          sys.read32(phys_pc + 0x08u));
    }
  }

  if (g_log_fmv_diagnostics && pc >= 0x8008B8E8u &&
      pc <= 0x8008B954u && cycles >= 399000000ull) {
    const u32 node_word0 = sys.read32(0x800A6518u);
    if ((node_word0 != 0u || pc == 0x8008B8FCu ||
         pc == 0x8008B94Cu) &&
        st.node_window < 192u) {
      ++st.node_window;
      LOG_WARN(
          "CPU: rr4-node window prev=0x%08X pc=0x%08X instr=0x%08X cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X s6=0x%08X sp=0x%08X "
          "ra=0x%08X "
          "node[0]=0x%08X q0=0x%08X q1=0x%08X low[10]=0x%08X "
          "low[18]=0x%08X",
          prev, pc, instruction,
          static_cast<unsigned long long>(cycles), regs[2], regs[3], regs[4],
          regs[5], regs[6], regs[7], regs[16], regs[17], regs[18], regs[19],
          regs[22], regs[29], regs[31], node_word0, sys.read32(0x800A5480u),
          sys.read32(0x800A5488u), sys.read32(0x10u), sys.read32(0x18u));
    }
  }

  if (g_log_fmv_diagnostics && cycles >= 930000000ull &&
      ((pc >= 0x8008B8E8u && pc <= 0x8008B95Cu) ||
       (pc >= 0x800970B0u && pc <= 0x80097110u))) {
    if (!st.late_code_dumped) {
      st.late_code_dumped = true;
      const u32 low_b0 = sys.read32(0x000000B0u);
      const u32 low_b4 = sys.read32(0x000000B4u);
      const u32 low_b8 = sys.read32(0x000000B8u);
      const u32 low_bc = sys.read32(0x000000BCu);
      const u32 low_c0 = sys.read32(0x000000C0u);
      LOG_WARN(
          "CPU: rr4-late code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x0008B8E8u, sys.read32(0x8008B8E8u), 0x0008B8ECu,
          sys.read32(0x8008B8ECu), 0x0008B8F0u, sys.read32(0x8008B8F0u),
          0x0008B8F4u, sys.read32(0x8008B8F4u), 0x0008B8F8u,
          sys.read32(0x8008B8F8u));
      LOG_WARN(
          "CPU: rr4-late code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x0008B8FCu, sys.read32(0x8008B8FCu), 0x0008B900u,
          sys.read32(0x8008B900u), 0x0008B904u, sys.read32(0x8008B904u),
          0x0008B908u, sys.read32(0x8008B908u), 0x0008B90Cu,
          sys.read32(0x8008B90Cu));
      LOG_WARN(
          "CPU: rr4-late code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x0008B910u, sys.read32(0x8008B910u), 0x0008B914u,
          sys.read32(0x8008B914u), 0x0008B918u, sys.read32(0x8008B918u),
          0x0008B91Cu, sys.read32(0x8008B91Cu), 0x0008B920u,
          sys.read32(0x8008B920u));
      LOG_WARN(
          "CPU: rr4-late code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x0008B924u, sys.read32(0x8008B924u), 0x0008B928u,
          sys.read32(0x8008B928u), 0x0008B92Cu, sys.read32(0x8008B92Cu),
          0x0008B930u, sys.read32(0x8008B930u), 0x0008B934u,
          sys.read32(0x8008B934u));
      LOG_WARN(
          "CPU: rr4-late code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x0008B938u, sys.read32(0x8008B938u), 0x0008B93Cu,
          sys.read32(0x8008B93Cu), 0x0008B940u, sys.read32(0x8008B940u),
          0x0008B944u, sys.read32(0x8008B944u), 0x0008B948u,
          sys.read32(0x8008B948u));
      LOG_WARN(
          "CPU: rr4-late code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x0008B94Cu, sys.read32(0x8008B94Cu), 0x0008B950u,
          sys.read32(0x8008B950u), 0x0008B954u, sys.read32(0x8008B954u),
          0x0008B958u, sys.read32(0x8008B958u), 0x0008B95Cu,
          sys.read32(0x8008B95Cu));
      LOG_WARN(
          "CPU: rr4-late code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x000970B0u, sys.read32(0x800970B0u), 0x000970B4u,
          sys.read32(0x800970B4u), 0x000970B8u, sys.read32(0x800970B8u),
          0x000970BCu, sys.read32(0x800970BCu), 0x000970C0u,
          sys.read32(0x800970C0u));
      LOG_WARN(
          "CPU: rr4-late code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x000970C4u, sys.read32(0x800970C4u), 0x000970C8u,
          sys.read32(0x800970C8u), 0x000970CCu, sys.read32(0x800970CCu),
          0x000970D0u, sys.read32(0x800970D0u), 0x000970D4u,
          sys.read32(0x800970D4u));
      LOG_WARN(
          "CPU: rr4-late lowvec %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x000000B0u, low_b0, 0x000000B4u, low_b4, 0x000000B8u, low_b8,
          0x000000BCu, low_bc, 0x000000C0u, low_c0);
      if (low_b0 == 0u && low_b4 == 0u && low_b8 == 0u && low_bc == 0u &&
          low_c0 == 0u) {
        sys.debug_log_recent_ram_writes(0x000000B0u, 0x20u, "CPU");
      }
    }
    if (st.late_low_path < 192u) {
      ++st.late_low_path;
      LOG_WARN(
          "CPU: rr4-late path prev=0x%08X pc=0x%08X instr=0x%08X cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "t0=0x%08X t1=0x%08X t2=0x%08X "
          "s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X s6=0x%08X sp=0x%08X "
          "ra=0x%08X "
          "node0=0x%08X node4=0x%08X node8=0x%08X low10=0x%08X "
          "low18=0x%08X "
          "lowB0=0x%08X lowB4=0x%08X lowB8=0x%08X lowBC=0x%08X "
          "lowC0=0x%08X "
          "sr=0x%08X cause=0x%08X irq=%d",
          prev, pc, instruction,
          static_cast<unsigned long long>(cycles), regs[2], regs[3], regs[4],
          regs[5], regs[6], regs[7], regs[8], regs[9], regs[10], regs[16],
          regs[17], regs[18], regs[19], regs[22], regs[29], regs[31],
          sys.read32(0x800A6518u), sys.read32(0x800A651Cu),
          sys.read32(0x800A6520u), sys.read32(0x10u), sys.read32(0x18u),
          sys.read32(0x000000B0u), sys.read32(0x000000B4u),
          sys.read32(0x000000B8u), sys.read32(0x000000BCu),
          sys.read32(0x000000C0u), cop0_sr, cop0_cause,
          irq_pending ? 1 : 0);
    }
  }

  if (g_log_fmv_diagnostics && pc >= 0x80116CC0u &&
      pc <= 0x80116D20u) {
    if (st.decomp_step < 768u) {
      ++st.decomp_step;
      const u32 src0 = regs[4];
      const u32 dst0 = regs[5];
      const u32 bitp = regs[8];
      LOG_WARN(
          "CPU: rr4-decomp step n=%u prev=0x%08X pc=0x%08X instr=0x%08X "
          "cyc=%llu "
          "a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X t0=0x%08X t1=0x%08X "
          "t2=0x%08X t3=0x%08X v0=0x%08X v1=0x%08X s0=0x%08X s1=0x%08X "
          "src0=0x%08X src4=0x%08X bit0=0x%08X bit4=0x%08X dst0=0x%08X "
          "dst4=0x%08X",
          st.decomp_step, prev, pc, instruction,
          static_cast<unsigned long long>(cycles), regs[4], regs[5],
          regs[6], regs[7], regs[8], regs[9], regs[10], regs[11], regs[2],
          regs[3], regs[16], regs[17], sys.read32(src0),
          sys.read32(src0 + 4u), sys.read32(bitp), sys.read32(bitp + 4u),
          sys.read32(dst0), sys.read32(dst0 + 4u));
    }
    if (!st.lowclr_entry_logged) {
      st.lowclr_entry_logged = true;
      LOG_WARN(
          "CPU: rr4-lowclr entry prev_pc=0x%08X pc=0x%08X instr=0x%08X "
          "cyc=%llu "
          "sp=0x%08X ra=0x%08X v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X "
          "a2=0x%08X a3=0x%08X "
          "t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X s0=0x%08X s1=0x%08X",
          prev, pc, instruction,
          static_cast<unsigned long long>(cycles), regs[29], regs[31],
          regs[2], regs[3], regs[4], regs[5], regs[6], regs[7], regs[8],
          regs[9], regs[10], regs[11], regs[16], regs[17]);
      LOG_WARN(
          "CPU: rr4-lowclr code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x00116CC0u, sys.read32(0x00116CC0u), 0x00116CC4u,
          sys.read32(0x00116CC4u), 0x00116CC8u, sys.read32(0x00116CC8u),
          0x00116CCCu, sys.read32(0x00116CCCu), 0x00116CD0u,
          sys.read32(0x00116CD0u));
      LOG_WARN(
          "CPU: rr4-lowclr code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x00116CD4u, sys.read32(0x00116CD4u), 0x00116CD8u,
          sys.read32(0x00116CD8u), 0x00116CDCu, sys.read32(0x00116CDCu),
          0x00116CE0u, sys.read32(0x00116CE0u), 0x00116CE4u,
          sys.read32(0x00116CE4u));
      LOG_WARN(
          "CPU: rr4-lowclr code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x00116CE8u, sys.read32(0x00116CE8u), 0x00116CECu,
          sys.read32(0x00116CECu), 0x00116CF0u, sys.read32(0x00116CF0u),
          0x00116CF4u, sys.read32(0x00116CF4u), 0x00116CF8u,
          sys.read32(0x00116CF8u));
      LOG_WARN(
          "CPU: rr4-lowclr code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x00116CFCu, sys.read32(0x00116CFCu), 0x00116D00u,
          sys.read32(0x00116D00u), 0x00116D04u, sys.read32(0x00116D04u),
          0x00116D08u, sys.read32(0x00116D08u), 0x00116D0Cu,
          sys.read32(0x00116D0Cu));
      LOG_WARN(
          "CPU: rr4-lowclr code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x00116D10u, sys.read32(0x00116D10u), 0x00116D14u,
          sys.read32(0x00116D14u), 0x00116D18u, sys.read32(0x00116D18u),
          0x00116D1Cu, sys.read32(0x00116D1Cu), 0x00116D20u,
          sys.read32(0x00116D20u));
    }
    const u32 a1_phys = regs[5] & 0x1FFFFFFFu;
    if (!st.lowclr_lowdest_logged && a1_phys < 0x00000200u &&
        pc >= 0x80116D0Cu && pc <= 0x80116D14u) {
      st.lowclr_lowdest_logged = true;
      LOG_WARN(
          "CPU: rr4-lowclr lowdest prev=0x%08X pc=0x%08X instr=0x%08X "
          "cyc=%llu "
          "a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X t0=0x%08X t1=0x%08X "
          "t2=0x%08X t3=0x%08X v0=0x%08X v1=0x%08X ra=0x%08X sp=0x%08X",
          prev, pc, instruction,
          static_cast<unsigned long long>(cycles), regs[4], regs[5], regs[6],
          regs[7], regs[8], regs[9], regs[10], regs[11], regs[2], regs[3],
          regs[31], regs[29]);
      LOG_WARN(
          "CPU: rr4-lowclr src %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          (regs[4] - 0x08u) & 0x1FFFFFFFu, sys.read32(regs[4] - 0x08u),
          (regs[4] - 0x04u) & 0x1FFFFFFFu, sys.read32(regs[4] - 0x04u),
          (regs[4] + 0x00u) & 0x1FFFFFFFu, sys.read32(regs[4] + 0x00u),
          (regs[4] + 0x04u) & 0x1FFFFFFFu, sys.read32(regs[4] + 0x04u),
          (regs[4] + 0x08u) & 0x1FFFFFFFu, sys.read32(regs[4] + 0x08u));
      LOG_WARN(
          "CPU: rr4-lowclr dst %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          (regs[5] - 0x08u) & 0x1FFFFFFFu, sys.read32(regs[5] - 0x08u),
          (regs[5] - 0x04u) & 0x1FFFFFFFu, sys.read32(regs[5] - 0x04u),
          (regs[5] + 0x00u) & 0x1FFFFFFFu, sys.read32(regs[5] + 0x00u),
          (regs[5] + 0x04u) & 0x1FFFFFFFu, sys.read32(regs[5] + 0x04u),
          (regs[5] + 0x08u) & 0x1FFFFFFFu, sys.read32(regs[5] + 0x08u));
      sys.debug_log_recent_ram_writes(regs[5], 0x20u, "CPU");
    }
  }

  if (g_log_fmv_diagnostics && cycles >= 900000000ull &&
      pc >= 0x80115480u && pc <= 0x80115530u) {
    if (st.wait_log < 256u) {
      ++st.wait_log;
      const u32 phys_pc = pc & 0x1FFFFFFFu;
      LOG_WARN(
          "CPU: RR4 wait pc=0x%08X prev=0x%08X instr=0x%08X cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X s0=0x%08X s1=0x%08X "
          "s2=0x%08X s3=0x%08X sp=0x%08X ra=0x%08X sr=0x%08X "
          "cause=0x%08X",
          pc, prev, instruction,
          static_cast<unsigned long long>(cycles), regs[2], regs[3], regs[4],
          regs[5], regs[6], regs[7], regs[8], regs[9], regs[10], regs[11],
          regs[16], regs[17], regs[18], regs[19], regs[29], regs[31],
          cop0_sr, cop0_cause);
      LOG_WARN(
          "CPU: RR4 wait code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          phys_pc - 0x10u, sys.read32(phys_pc - 0x10u), phys_pc - 0x0Cu,
          sys.read32(phys_pc - 0x0Cu), phys_pc - 0x08u,
          sys.read32(phys_pc - 0x08u), phys_pc - 0x04u,
          sys.read32(phys_pc - 0x04u), phys_pc + 0x00u,
          sys.read32(phys_pc + 0x00u), phys_pc + 0x04u,
          sys.read32(phys_pc + 0x04u), phys_pc + 0x08u,
          sys.read32(phys_pc + 0x08u), phys_pc + 0x0Cu,
          sys.read32(phys_pc + 0x0Cu), phys_pc + 0x10u,
          sys.read32(phys_pc + 0x10u));
      LOG_WARN(
          "CPU: RR4 wait mem str=%08X %08X %08X %08X node=%08X %08X %08X "
          "obj=%08X %08X %08X %08X %08X %08X %08X %08X "
          "mdec_status=0x%08X dma0=%08X/%08X/%08X dma1=%08X/%08X/%08X "
          "dpcr=0x%08X dicr=0x%08X",
          sys.read32(0x00141BF4u), sys.read32(0x00141BF8u),
          sys.read32(0x00141BFCu), sys.read32(0x00141C00u),
          sys.read32(0x00110400u), sys.read32(0x00110404u),
          sys.read32(0x00110408u), sys.read32(0x001281CCu),
          sys.read32(0x001281D0u), sys.read32(0x001281D4u),
          sys.read32(0x001281D8u), sys.read32(0x001281DCu),
          sys.read32(0x001281E0u), sys.read32(0x001281E4u),
          sys.read32(0x001281E8u), sys.read32(0x1F801824u),
          sys.read32(0x1F801080u), sys.read32(0x1F801084u),
          sys.read32(0x1F801088u), sys.read32(0x1F801090u),
          sys.read32(0x1F801094u), sys.read32(0x1F801098u),
          sys.read32(0x1F8010F0u), sys.read32(0x1F8010F4u));
      LOG_WARN(
          "CPU: RR4 wait caller %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x00114E90u, sys.read32(0x00114E90u), 0x00114E94u,
          sys.read32(0x00114E94u), 0x00114E98u,
          sys.read32(0x00114E98u), 0x00114E9Cu,
          sys.read32(0x00114E9Cu), 0x00114EA0u,
          sys.read32(0x00114EA0u), 0x00114EA4u,
          sys.read32(0x00114EA4u), 0x00114EA8u,
          sys.read32(0x00114EA8u), 0x00114EACu,
          sys.read32(0x00114EACu));
    }
  }

  if (g_log_fmv_diagnostics && cycles >= 880000000ull &&
      ((pc >= 0x8008BDE0u && pc <= 0x8008BE40u) ||
       (pc >= 0x801150D0u && pc <= 0x801151C0u))) {
    if (st.dma_callback < 512u) {
      ++st.dma_callback;
      LOG_WARN(
          "CPU: RR4 dma-callback pc=0x%08X prev=0x%08X instr=0x%08X "
          "cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "t0=0x%08X t1=0x%08X s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X "
          "sp=0x%08X ra=0x%08X sr=0x%08X cause=0x%08X "
          "istat=0x%08X imask=0x%08X dicr=0x%08X "
          "obj=%08X/%08X/%08X/%08X/%08X/%08X",
          pc, prev, instruction,
          static_cast<unsigned long long>(cycles), regs[2], regs[3], regs[4],
          regs[5], regs[6], regs[7], regs[8], regs[9], regs[16], regs[17],
          regs[18], regs[19], regs[29], regs[31], cop0_sr, cop0_cause,
          sys.read32(0x1F801070u), sys.read32(0x1F801074u),
          sys.read32(0x1F8010F4u), sys.read32(0x001281CCu),
          sys.read32(0x001281D0u), sys.read32(0x001281D4u),
          sys.read32(0x001281D8u), sys.read32(0x001281ECu),
          sys.read32(0x001281EEu));
    }
  }

  if (g_log_fmv_diagnostics &&
      ((regs[10] == 0x000000B0u &&
        (regs[9] == 0x00000017u || regs[9] == 0x00000018u)) ||
       pc == 0x000000B0u || pc == 0x000000C0u) &&
      cycles >= 390000000ull) {
    if (st.bios_trampoline < 192u) {
      ++st.bios_trampoline;
      LOG_WARN(
          "CPU: bios-call pc=0x%08X prev=0x%08X instr=0x%08X cyc=%llu "
          "t1=0x%08X t2=0x%08X v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X "
          "a2=0x%08X a3=0x%08X s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X "
          "sp=0x%08X ra=0x%08X sr=0x%08X cause=0x%08X irq=%d",
          pc, prev, instruction,
          static_cast<unsigned long long>(cycles), regs[9], regs[10], regs[2],
          regs[3], regs[4], regs[5], regs[6], regs[7], regs[16], regs[17],
          regs[18], regs[19], regs[29], regs[31], cop0_sr, cop0_cause,
          irq_pending ? 1 : 0);
    }
  }

  if (g_log_fmv_diagnostics && pc >= 0x800A5480u &&
      pc <= 0x800A54C0u && cycles >= 780000000ull) {
    if (!st.callback_code_dumped) {
      st.callback_code_dumped = true;
      LOG_WARN(
          "CPU: rr4-callback code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x000A5480u, sys.read32(0x800A5480u), 0x000A5484u,
          sys.read32(0x800A5484u), 0x000A5488u, sys.read32(0x800A5488u),
          0x000A548Cu, sys.read32(0x800A548Cu), 0x000A5490u,
          sys.read32(0x800A5490u));
      LOG_WARN(
          "CPU: rr4-callback code %08X=%08X %08X=%08X %08X=%08X %08X=%08X "
          "%08X=%08X",
          0x000A5494u, sys.read32(0x800A5494u), 0x000A5498u,
          sys.read32(0x800A5498u), 0x000A549Cu, sys.read32(0x800A549Cu),
          0x000A54A0u, sys.read32(0x800A54A0u), 0x000A54A4u,
          sys.read32(0x800A54A4u));
    }
    if (st.callback_log < 192u) {
      ++st.callback_log;
      LOG_WARN(
          "CPU: rr4-callback prev=0x%08X pc=0x%08X instr=0x%08X cyc=%llu "
          "v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X s6=0x%08X sp=0x%08X "
          "ra=0x%08X "
          "node0=0x%08X node4=0x%08X node8=0x%08X",
          prev, pc, instruction,
          static_cast<unsigned long long>(cycles), regs[2], regs[3], regs[4],
          regs[5], regs[6], regs[7], regs[16], regs[17], regs[18], regs[19],
          regs[22], regs[29], regs[31], sys.read32(0x800A6518u),
          sys.read32(0x800A651Cu), sys.read32(0x800A6520u));
    }
  }

  if (g_log_fmv_diagnostics && cycles >= 835000000ull &&
      ((pc >= 0x8008B8F0u && pc <= 0x8008B954u) ||
       (pc >= 0x00000DE0u && pc <= 0x00000E30u))) {
    if (st.fault_window < 192u) {
      ++st.fault_window;
      const u32 node_head = sys.read32(0x800A6518u);
      const u32 q0 = sys.read32(0x800A5480u);
      const u32 q1 = sys.read32(0x800A5488u);
      const u32 low0 = sys.read32(0x00000000u);
      const u32 low8 = sys.read32(0x00000008u);
      const u32 low10 = sys.read32(0x00000010u);
      const u32 low18 = sys.read32(0x00000018u);
      LOG_WARN(
          "CPU: rr4 fault-window prev=0x%08X pc=0x%08X instr=0x%08X cyc=%llu "
          "sr=0x%08X cause=0x%08X irq=%d sp=0x%08X ra=0x%08X "
          "s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X s6=0x%08X "
          "a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
          "node=0x%08X q0=0x%08X q1=0x%08X low[0]=0x%08X low[8]=0x%08X "
          "low[10]=0x%08X low[18]=0x%08X",
          prev, pc, instruction,
          static_cast<unsigned long long>(cycles), cop0_sr, cop0_cause,
          irq_pending ? 1 : 0, regs[29], regs[31], regs[16], regs[17],
          regs[18], regs[19], regs[22], regs[4], regs[5], regs[6], regs[7],
          node_head, q0, q1, low0, low8, low10, low18);
    }
  }

  if (cpu_diag) {
    if (g_log_fmv_diagnostics && pc >= 0x80115240u &&
        pc <= 0x80115270u) {
      if (!st.lowclr_caller) {
        st.lowclr_caller = 1;
        LOG_WARN(
            "CPU: rr4-lowclr caller prev_pc=0x%08X pc=0x%08X instr=0x%08X "
            "cyc=%llu "
            "sp=0x%08X ra=0x%08X v0=0x%08X v1=0x%08X a0=0x%08X a1=0x%08X "
            "a2=0x%08X a3=0x%08X "
            "s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X",
            prev, pc, instruction,
            static_cast<unsigned long long>(cycles), regs[29], regs[31],
            regs[2], regs[3], regs[4], regs[5], regs[6], regs[7], regs[16],
            regs[17], regs[18], regs[19]);
        LOG_WARN(
            "CPU: rr4-lowclr caller code %08X=%08X %08X=%08X %08X=%08X "
            "%08X=%08X %08X=%08X",
            0x00115240u, sys.read32(0x00115240u), 0x00115244u,
            sys.read32(0x00115244u), 0x00115248u, sys.read32(0x00115248u),
            0x0011524Cu, sys.read32(0x0011524Cu), 0x00115250u,
            sys.read32(0x00115250u));
        LOG_WARN(
            "CPU: rr4-lowclr caller code %08X=%08X %08X=%08X %08X=%08X "
            "%08X=%08X %08X=%08X",
            0x00115254u, sys.read32(0x00115254u), 0x00115258u,
            sys.read32(0x00115258u), 0x0011525Cu, sys.read32(0x0011525Cu),
            0x00115260u, sys.read32(0x00115260u), 0x00115264u,
            sys.read32(0x00115264u));
        LOG_WARN(
            "CPU: rr4-lowclr caller code %08X=%08X %08X=%08X %08X=%08X "
            "%08X=%08X %08X=%08X",
            0x00115268u, sys.read32(0x00115268u), 0x0011526Cu,
            sys.read32(0x0011526Cu), 0x00115270u, sys.read32(0x00115270u),
            0x00115274u, sys.read32(0x00115274u), 0x00115278u,
            sys.read32(0x00115278u));
      }
    }
  }
}

void on_op_lw(Rr4DiagState& st, u32 pc, u32 instruction, u32 addr,
              const u32* regs, u64 cycles, u32 cop0_sr, u32 cop0_cause,
              bool irq_pending) {
  const u32 phys = addr & 0x1FFFFFFFu;
  if (g_log_fmv_diagnostics && phys == 0x000A6518u &&
      cycles >= 835000000ull) {
    if (st.node_lw_fault < 160u) {
      ++st.node_lw_fault;
      LOG_WARN(
          "CPU: rr4-node LW-fault pc=0x%08X instr=0x%08X rs=r%u base=0x%08X "
          "rt=r%u imm=%d addr=0x%08X sr=0x%08X cause=0x%08X irq=%d "
          "s1=0x%08X s2=0x%08X s3=0x%08X s6=0x%08X ra=0x%08X",
          pc, instruction,
          static_cast<unsigned>(instr_rs(instruction)),
          regs[instr_rs(instruction)],
          static_cast<unsigned>(instr_rt(instruction)),
          static_cast<int>(instr_simm(instruction)), addr,
          cop0_sr, cop0_cause, irq_pending ? 1 : 0, regs[17],
          regs[18], regs[19], regs[22], regs[31]);
    }
  }
}

void on_op_sb(Rr4DiagState& st, u32 pc, u32 instruction, u32 addr,
              const u32* regs, u32 cop0_sr, u32 cop0_cause,
              bool irq_pending) {
  const u32 phys = addr & 0x1FFFFFFFu;
  if (g_log_fmv_diagnostics && phys >= 0x000A6500u && phys < 0x000A6540u) {
    if (st.node_sb < 128u) {
      ++st.node_sb;
      LOG_WARN(
          "CPU: rr4-node SB pc=0x%08X instr=0x%08X rs=r%u base=0x%08X "
          "rt=r%u val=0x%08X imm=%d addr=0x%08X",
          pc, instruction, static_cast<unsigned>(instr_rs(instruction)),
          regs[instr_rs(instruction)],
          static_cast<unsigned>(instr_rt(instruction)),
          regs[instr_rt(instruction)],
          static_cast<int>(instr_simm(instruction)), addr);
    }
  }
  if (g_log_fmv_diagnostics && phys >= 0x000A6540u && phys < 0x000A6550u) {
    if (st.bridge_sb < 64u) {
      ++st.bridge_sb;
      LOG_WARN(
          "CPU: rr4-bridge SB pc=0x%08X instr=0x%08X rs=r%u base=0x%08X "
          "rt=r%u val=0x%08X imm=%d addr=0x%08X",
          pc, instruction, static_cast<unsigned>(instr_rs(instruction)),
          regs[instr_rs(instruction)],
          static_cast<unsigned>(instr_rt(instruction)),
          regs[instr_rt(instruction)],
          static_cast<int>(instr_simm(instruction)), addr);
    }
  }
}

void on_op_sh(Rr4DiagState& st, u32 pc, u32 instruction, u32 addr,
              const u32* regs) {
  const u32 phys = addr & 0x1FFFFFFFu;
  if (g_log_fmv_diagnostics && phys >= 0x000A6500u && phys < 0x000A6540u) {
    if (st.node_sh < 128u) {
      ++st.node_sh;
      LOG_WARN(
          "CPU: rr4-node SH pc=0x%08X instr=0x%08X rs=r%u base=0x%08X "
          "rt=r%u val=0x%08X imm=%d addr=0x%08X",
          pc, instruction, static_cast<unsigned>(instr_rs(instruction)),
          regs[instr_rs(instruction)],
          static_cast<unsigned>(instr_rt(instruction)),
          regs[instr_rt(instruction)],
          static_cast<int>(instr_simm(instruction)), addr);
    }
  }
  if (g_log_fmv_diagnostics && phys >= 0x000A6540u && phys < 0x000A6550u) {
    if (st.bridge_sh < 64u) {
      ++st.bridge_sh;
      LOG_WARN(
          "CPU: rr4-bridge SH pc=0x%08X instr=0x%08X rs=r%u base=0x%08X "
          "rt=r%u val=0x%08X imm=%d addr=0x%08X",
          pc, instruction, static_cast<unsigned>(instr_rs(instruction)),
          regs[instr_rs(instruction)],
          static_cast<unsigned>(instr_rt(instruction)),
          regs[instr_rt(instruction)],
          static_cast<int>(instr_simm(instruction)), addr);
    }
  }
}

void on_op_sw(Rr4DiagState& st, u32 pc, u32 instruction, u32 addr,
              u32 store_val, const u32* regs, u64 cycles, u32 cop0_sr,
              u32 cop0_cause, bool irq_pending) {
  const u32 phys = addr & 0x1FFFFFFFu;
  if (g_log_fmv_diagnostics && phys < 0x00000040u) {
    if (st.low_slot_sw < 96u) {
      ++st.low_slot_sw;
      LOG_WARN(
          "CPU: low-slot SW pc=0x%08X instr=0x%08X rs=r%u base=0x%08X "
          "rt=r%u val=0x%08X imm=%d addr=0x%08X sr=0x%08X cause=0x%08X "
          "irq=%d "
          "s1=0x%08X s2=0x%08X s3=0x%08X ra=0x%08X",
          pc, instruction, static_cast<unsigned>(instr_rs(instruction)),
          regs[instr_rs(instruction)],
          static_cast<unsigned>(instr_rt(instruction)),
          regs[instr_rt(instruction)],
          static_cast<int>(instr_simm(instruction)), addr, cop0_sr,
          cop0_cause, irq_pending ? 1 : 0, regs[17], regs[18], regs[19],
          regs[31]);
    }
    if ((phys == 0x00000010u || phys == 0x00000018u) &&
        cycles >= 1000000000ull) {
      if (st.low_slot_sw_late < 96u) {
        ++st.low_slot_sw_late;
        LOG_WARN(
            "CPU: low-slot SW-late pc=0x%08X instr=0x%08X rs=r%u base=0x%08X "
            "rt=r%u val=0x%08X imm=%d addr=0x%08X sr=0x%08X cause=0x%08X "
            "irq=%d "
            "s1=0x%08X s2=0x%08X s3=0x%08X ra=0x%08X",
            pc, instruction, static_cast<unsigned>(instr_rs(instruction)),
            regs[instr_rs(instruction)],
            static_cast<unsigned>(instr_rt(instruction)),
            regs[instr_rt(instruction)],
            static_cast<int>(instr_simm(instruction)), addr, cop0_sr,
            cop0_cause, irq_pending ? 1 : 0, regs[17], regs[18], regs[19],
            regs[31]);
      }
    }
  }
  if (g_log_fmv_diagnostics && phys >= 0x000000B0u && phys <= 0x000000C0u) {
    if (st.low_vec_sw < 128u) {
      ++st.low_vec_sw;
      LOG_WARN(
          "CPU: low-vec SW pc=0x%08X instr=0x%08X rs=r%u base=0x%08X "
          "rt=r%u val=0x%08X imm=%d addr=0x%08X sr=0x%08X cause=0x%08X "
          "irq=%d "
          "a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X ra=0x%08X",
          pc, instruction, static_cast<unsigned>(instr_rs(instruction)),
          regs[instr_rs(instruction)],
          static_cast<unsigned>(instr_rt(instruction)),
          regs[instr_rt(instruction)],
          static_cast<int>(instr_simm(instruction)), addr, cop0_sr,
          cop0_cause, irq_pending ? 1 : 0, regs[4], regs[5], regs[6],
          regs[7], regs[31]);
    }
  }
  const bool low_slot_focus =
      (phys == 0x00000010u && store_val == 0x800A6518u) ||
      (phys == 0x00000018u &&
       (store_val == 0x00000001u || store_val == 0x801A9530u));
  if (low_slot_focus) {
    if (st.low_slot_sw_focus < 64u) {
      ++st.low_slot_sw_focus;
      LOG_WARN(
          "CPU: low-slot SW-focus pc=0x%08X instr=0x%08X rs=r%u base=0x%08X "
          "rt=r%u val=0x%08X imm=%d addr=0x%08X sr=0x%08X cause=0x%08X "
          "irq=%d "
          "v0=0x%08X v1=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X s6=0x%08X "
          "ra=0x%08X",
          pc, instruction, static_cast<unsigned>(instr_rs(instruction)),
          regs[instr_rs(instruction)],
          static_cast<unsigned>(instr_rt(instruction)),
          regs[instr_rt(instruction)],
          static_cast<int>(instr_simm(instruction)), addr, cop0_sr,
          cop0_cause, irq_pending ? 1 : 0, regs[2], regs[3], regs[17],
          regs[18], regs[19], regs[22], regs[31]);
    }
  }
  if (g_log_fmv_diagnostics && phys >= 0x000A6500u && phys < 0x000A6540u) {
    if (st.node_sw < 256u) {
      ++st.node_sw;
      LOG_WARN(
          "CPU: rr4-node SW pc=0x%08X instr=0x%08X rs=r%u base=0x%08X "
          "rt=r%u val=0x%08X imm=%d addr=0x%08X "
          "s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X sp=0x%08X ra=0x%08X "
          "sr=0x%08X cause=0x%08X irq=%d",
          pc, instruction, static_cast<unsigned>(instr_rs(instruction)),
          regs[instr_rs(instruction)],
          static_cast<unsigned>(instr_rt(instruction)),
          regs[instr_rt(instruction)],
          static_cast<int>(instr_simm(instruction)), addr,
          regs[16], regs[17], regs[18], regs[19], regs[29], regs[31],
          cop0_sr, cop0_cause, irq_pending ? 1 : 0);
    }
    if (phys == 0x000A6518u && cycles >= 780000000ull) {
      if (st.node_sw_late < 128u) {
        ++st.node_sw_late;
        LOG_WARN(
            "CPU: rr4-node SW-late pc=0x%08X instr=0x%08X rs=r%u base=0x%08X "
            "rt=r%u val=0x%08X imm=%d addr=0x%08X sr=0x%08X cause=0x%08X "
            "irq=%d "
            "s1=0x%08X s2=0x%08X s3=0x%08X ra=0x%08X",
            pc, instruction, static_cast<unsigned>(instr_rs(instruction)),
            regs[instr_rs(instruction)],
            static_cast<unsigned>(instr_rt(instruction)),
            regs[instr_rt(instruction)],
            static_cast<int>(instr_simm(instruction)), addr, cop0_sr,
            cop0_cause, irq_pending ? 1 : 0, regs[17], regs[18], regs[19],
            regs[31]);
      }
    }
    if (phys == 0x000A6518u && cycles >= 835000000ull) {
      if (st.node_sw_fault < 160u) {
        ++st.node_sw_fault;
        LOG_WARN(
            "CPU: rr4-node SW-fault pc=0x%08X instr=0x%08X rs=r%u base=0x%08X "
            "rt=r%u val=0x%08X imm=%d addr=0x%08X sr=0x%08X cause=0x%08X "
            "irq=%d "
            "s1=0x%08X s2=0x%08X s3=0x%08X s6=0x%08X ra=0x%08X",
            pc, instruction, static_cast<unsigned>(instr_rs(instruction)),
            regs[instr_rs(instruction)],
            static_cast<unsigned>(instr_rt(instruction)),
            regs[instr_rt(instruction)],
            static_cast<int>(instr_simm(instruction)), addr, cop0_sr,
            cop0_cause, irq_pending ? 1 : 0, regs[17], regs[18], regs[19],
            regs[22], regs[31]);
      }
    }
  }
  if (g_log_fmv_diagnostics && phys >= 0x000A6540u && phys < 0x000A6550u) {
    if (st.bridge_sw < 128u) {
      ++st.bridge_sw;
      LOG_WARN(
          "CPU: rr4-bridge SW pc=0x%08X instr=0x%08X rs=r%u base=0x%08X "
          "rt=r%u val=0x%08X imm=%d addr=0x%08X "
          "s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X sp=0x%08X ra=0x%08X "
          "sr=0x%08X cause=0x%08X irq=%d",
          pc, instruction, static_cast<unsigned>(instr_rs(instruction)),
          regs[instr_rs(instruction)],
          static_cast<unsigned>(instr_rt(instruction)),
          regs[instr_rt(instruction)],
          static_cast<int>(instr_simm(instruction)), addr,
          regs[16], regs[17], regs[18], regs[19], regs[29], regs[31],
          cop0_sr, cop0_cause, irq_pending ? 1 : 0);
    }
  }
}

void on_store16(Rr4DiagState& st, u32 pc, u32 addr, u16 value,
                const u32* regs, u64 cycles, System& sys) {
  if (g_log_fmv_diagnostics && pc >= 0x80116CC0u &&
      pc <= 0x80116D20u) {
    const u32 phys = addr & 0x1FFFFFFFu;
    if (phys < 0x00000200u ||
        (regs[5] >= 0x801FFF00u && regs[5] < 0x80200200u)) {
      if (st.lowclr_wrap < 96u) {
        ++st.lowclr_wrap;
        const u32 src_ptr = regs[8];
        const u32 src_phys = src_ptr & 0x1FFFFFFFu;
        const u32 dst_ptr = regs[5];
        LOG_WARN(
            "CPU: rr4-lowclr wrap pc=0x%08X instr=0x%08X addr=0x%08X "
            "phys=0x%08X "
            "val=0x%04X cyc=%llu a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X "
            "t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X v0=0x%08X v1=0x%08X "
            "src0=0x%08X src4=0x%08X low100=0x%08X low104=0x%08X "
            "low108=0x%08X "
            "ra=0x%08X sp=0x%08X",
            pc, sys.read32(pc), addr, phys,
            static_cast<unsigned>(value),
            static_cast<unsigned long long>(cycles),
            regs[4], regs[5], regs[6], regs[7], regs[8], regs[9], regs[10],
            regs[11], regs[2], regs[3], sys.read32(src_phys),
            sys.read32(src_phys + 4u), sys.read32(0x00000100u),
            sys.read32(0x00000104u), sys.read32(0x00000108u), regs[31],
            regs[29]);
      }
    }
    if (phys >= 0x00000100u && phys < 0x00000120u) {
      if (st.lowclr_store16 < 32u) {
        ++st.lowclr_store16;
        LOG_WARN(
            "CPU: rr4-lowclr store16 pc=0x%08X instr=0x%08X addr=0x%08X "
            "val=0x%04X cyc=%llu "
            "a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X t0=0x%08X t1=0x%08X "
            "t2=0x%08X t3=0x%08X sp=0x%08X ra=0x%08X",
            pc, sys.read32(pc), addr,
            static_cast<unsigned>(value),
            static_cast<unsigned long long>(cycles),
            regs[4], regs[5], regs[6], regs[7], regs[8], regs[9], regs[10],
            regs[11], regs[29], regs[31]);
      }
    }
  }
}

} // namespace rr4_diag
