#include "grim_live.h"
#include "system.h"
#include <algorithm>
#include <cstdio>

GrimLivenessConfig grim_live_config(bool game_disc) {
  GrimLivenessConfig c;
  c.exception_loop_frames = 45;
  c.stuck_exception_frames = 100; // clean boots idle up to 72 frames with a syscall's EPC set
  c.coverage_stall_frames = 150;
  c.frozen_frame_frames = 240;
  if (game_disc) {
    c.coverage_stall_frames *= 2u;
    c.stuck_exception_frames *= 2u;
    c.frozen_frame_frames *= 2u;
  }
  return c;
}

namespace {
const char *exception_words(u32 cause) {
  switch (cause) {
  case 4: return "Address-error (load)";
  case 5: return "Address-error (store)";
  case 6: return "Bus-error (instruction fetch)";
  case 7: return "Bus-error (data)";
  case 8: return "Syscall";
  case 9: return "Break";
  case 10: return "Reserved-instruction";
  case 11: return "Coprocessor-unusable";
  case 12: return "Overflow";
  case 13: return "Trap";
  default: return "Unknown";
  }
}

std::string hex_addr(u32 a) {
  char b[24];
  std::snprintf(b, sizeof(b), "%04X %04X", a >> 16, a & 0xFFFFu);
  return b;
}
} // namespace

GrimDeath grim_describe_death(const std::string &reason, u32 frame, double seconds, u32 pc,
                              u32 exception_cause, u32 epc, const GrimGenome *genome) {
  GrimDeath d;
  d.reason = reason;
  d.frame = frame;
  d.seconds = seconds;
  d.pc = pc;
  char buf[256];
  if (reason == "exception_loop") {
    d.headline = "Stuck in an exception loop";
    if (exception_cause != 0xFFFFFFFFu) {
      std::snprintf(buf, sizeof(buf),
                    "%s exception at %s, repeating with no progress.",
                    exception_words(exception_cause), hex_addr(epc).c_str());
    } else {
      std::snprintf(buf, sizeof(buf), "The CPU keeps taking the same exception.");
    }
    d.detail = buf;
    // The BIOS runs straight from ROM at 0xBFC00000 early on; a ROM gene that
    // patched exactly the faulting word is the likely culprit.
    const u32 phys = epc & 0x1FFFFFFFu;
    if (genome != nullptr && exception_cause != 0xFFFFFFFFu && phys >= 0x1FC00000u) {
      const u32 offset = phys - 0x1FC00000u;
      for (size_t gi = 0; gi < genome->genes.size() && d.culprit_gene < 0; ++gi) {
        for (const GrimRomPatch &p : genome->genes[gi].patches) {
          if (p.offset == (offset & ~3u)) {
            d.culprit_gene = static_cast<s32>(gi);
            break;
          }
        }
      }
    }
  } else if (reason == "coverage_stall") {
    d.headline = "Machine went inert";
    std::snprintf(buf, sizeof(buf),
                  "No picture, sound or device activity, with the CPU still running at %s.",
                  hex_addr(pc).c_str());
    d.detail = buf;
  } else if (reason == "frozen_frame") {
    d.headline = "Frozen picture";
    std::snprintf(buf, sizeof(buf),
                  "The image stopped changing and nothing is drawing. CPU at %s.",
                  hex_addr(pc).c_str());
    d.detail = buf;
  } else if (reason == "dead_audio") {
    d.headline = "Black screen, no sound";
    d.detail = "Nothing was ever drawn and nothing ever played.";
  } else {
    d.headline = "Dead";
    d.detail = reason;
  }
  return d;
}

bool grim_mercy_should_reroll(bool mercy_enabled, const GrimLiveStatus &status) {
  return mercy_enabled && status.dead && status.death.frame <= kGrimMercyWindowFrames;
}

GrimLiveWatch::GrimLiveWatch(const GrimGenome *genome, GrimLivenessConfig config)
    : config_(config), tracker_(config) {
  if (genome != nullptr) {
    genome_ = *genome;
  }
  telemetry_.set_tracks_execution(false);
}

void GrimLiveWatch::attach(System &sys) {
  sys.cpu().set_telemetry(&telemetry_);
  tap_.clear();
  sys.set_spu_audio_tap(&tap_);
  attached_ = true;
  std::lock_guard<std::mutex> lock(mutex_);
  status_.watching = true;
}

void GrimLiveWatch::detach(System &sys) {
  if (!attached_) {
    return;
  }
  sys.cpu().set_telemetry(nullptr);
  sys.set_spu_audio_tap(nullptr);
  attached_ = false;
  std::lock_guard<std::mutex> lock(mutex_);
  status_.watching = false;
}

void GrimLiveWatch::on_frame(System &sys) {
  if (dead_) {
    tap_.clear();
    return; // the verdict is fixed; keep the machine running, stop judging it
  }
  GrimFrameTelemetry f = telemetry_.end_frame(sys, tap_, false);
  tap_.clear();
  // The GUI does not sample the display each frame, so measure it here.
  const DisplaySampleInfo shown = sys.gpu().build_display_rgba(nullptr);
  f.fb_hash = shown.hash;
  f.display_enabled = shown.display_enabled;
  f.fb_lit = static_cast<u32>(std::min<u64>(shown.non_black_pixels, 0xFFFFFFFFull));
  ++frames_;

  const bool died = tracker_.update(f);
  const double seconds =
      static_cast<double>(telemetry_.total_cycles()) / static_cast<double>(psx::CPU_CLOCK_HZ);
  GrimLiveStatus next;
  next.watching = true;
  next.frames = frames_;
  next.seconds = seconds;
  if (died) {
    dead_ = true;
    next.dead = true;
    // The interpreter notes every exception; the recompiler takes them natively, so
    // fall back to the COP0 state at the end of the frame.
    const bool noted = telemetry_.last_exception_cause() != 0xFFFFFFFFu;
    next.death = grim_describe_death(
        tracker_.finish().reason, f.frame, seconds, sys.cpu().pc(),
        noted ? telemetry_.last_exception_cause() : ((f.cop0_cause >> 2) & 31u),
        noted ? telemetry_.last_exception_epc() : f.cop0_epc, &genome_);
  } else {
    // Whole-run audio gate: a machine that never drew and never made a sound.
    const GrimLiveness verdict = tracker_.finish();
    if (!verdict.alive) {
      dead_ = true;
      next.dead = true;
      next.death = grim_describe_death(verdict.reason, f.frame, seconds, sys.cpu().pc(),
                                       0xFFFFFFFFu, 0, &genome_);
    }
    next.silent = verdict.silent;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  status_ = next;
}

GrimLiveStatus GrimLiveWatch::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}
