#include "platform/grim_process.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <cerrno>
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#endif

#ifdef _WIN32
namespace {

std::wstring widen(const std::string &s) {
  if (s.empty()) {
    return std::wstring();
  }
  const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
  std::wstring w(static_cast<size_t>(n), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
  return w;
}

// Standard CommandLineToArgvW-compatible quoting.
std::wstring quote(const std::wstring &arg) {
  if (!arg.empty() && arg.find_first_of(L" \t\n\v\"") == std::wstring::npos) {
    return arg;
  }
  std::wstring out = L"\"";
  size_t backslashes = 0;
  for (const wchar_t c : arg) {
    if (c == L'\\') {
      ++backslashes;
      continue;
    }
    if (c == L'"') {
      out.append(backslashes * 2 + 1, L'\\');
    } else {
      out.append(backslashes, L'\\');
    }
    backslashes = 0;
    out.push_back(c);
  }
  out.append(backslashes * 2, L'\\');
  out.push_back(L'"');
  return out;
}

} // namespace

GrimChildResult grim_run_child(const std::string &exe, const std::vector<std::string> &args,
                               const std::string &output_path, double timeout_seconds) {
  GrimChildResult result;

  SECURITY_ATTRIBUTES sa{};
  sa.nLength = sizeof(sa);
  sa.bInheritHandle = TRUE;
  HANDLE out = CreateFileW(widen(output_path).c_str(), GENERIC_WRITE, FILE_SHARE_READ, &sa,
                           CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (out == INVALID_HANDLE_VALUE) {
    return result;
  }
  HANDLE in = CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa,
                          OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);

  std::wstring cmd = quote(widen(exe));
  for (const std::string &a : args) {
    cmd += L" " + quote(widen(a));
  }

  STARTUPINFOW si{};
  si.cb = sizeof(si);
  si.dwFlags = STARTF_USESTDHANDLES;
  si.hStdInput = in;
  si.hStdOutput = out;
  si.hStdError = out;
  PROCESS_INFORMATION pi{};

  // No crash dialogs: a crashing child must exit, not wait for a click.
  const UINT old_mode = SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
  const BOOL created = CreateProcessW(nullptr, cmd.data(), nullptr, nullptr, TRUE,
                                      CREATE_SUSPENDED | CREATE_NO_WINDOW, nullptr, nullptr,
                                      &si, &pi);
  SetErrorMode(old_mode);
  if (!created) {
    CloseHandle(out);
    if (in != INVALID_HANDLE_VALUE) {
      CloseHandle(in);
    }
    return result;
  }
  result.started = true;

  HANDLE job = CreateJobObjectW(nullptr, nullptr);
  if (job != nullptr) {
    JOBOBJECT_EXTENDED_LIMIT_INFORMATION info{};
    info.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    SetInformationJobObject(job, JobObjectExtendedLimitInformation, &info, sizeof(info));
    AssignProcessToJobObject(job, pi.hProcess);
  }
  ResumeThread(pi.hThread);

  const DWORD wait_ms =
      timeout_seconds > 0.0 ? static_cast<DWORD>(timeout_seconds * 1000.0) : INFINITE;
  if (WaitForSingleObject(pi.hProcess, wait_ms) == WAIT_TIMEOUT) {
    result.timed_out = true;
    if (job != nullptr) {
      TerminateJobObject(job, 1);
    } else {
      TerminateProcess(pi.hProcess, 1);
    }
    WaitForSingleObject(pi.hProcess, 5000);
  }
  DWORD code = 0;
  if (GetExitCodeProcess(pi.hProcess, &code)) {
    result.exit_code = static_cast<int>(code);
    // NTSTATUS exception codes (access violation, stack overflow, ...).
    result.crashed = !result.timed_out && (code & 0xF0000000u) == 0xC0000000u;
  }
  CloseHandle(pi.hThread);
  CloseHandle(pi.hProcess);
  if (job != nullptr) {
    CloseHandle(job); // kill-on-close reaps any stragglers
  }
  CloseHandle(out);
  if (in != INVALID_HANDLE_VALUE) {
    CloseHandle(in);
  }
  return result;
}

std::string grim_self_exe_path(const std::string &argv0) {
  wchar_t buf[MAX_PATH * 4];
  const DWORD cap = static_cast<DWORD>(sizeof(buf) / sizeof(buf[0]));
  const DWORD n = GetModuleFileNameW(nullptr, buf, cap);
  if (n == 0 || n >= cap) {
    return argv0;
  }
  const int bytes =
      WideCharToMultiByte(CP_UTF8, 0, buf, static_cast<int>(n), nullptr, 0, nullptr, nullptr);
  std::string s(static_cast<size_t>(bytes), '\0');
  WideCharToMultiByte(CP_UTF8, 0, buf, static_cast<int>(n), s.data(), bytes, nullptr, nullptr);
  return s;
}

#else // POSIX

GrimChildResult grim_run_child(const std::string &exe, const std::vector<std::string> &args,
                               const std::string &output_path, double timeout_seconds) {
  GrimChildResult result;
  const int out_fd = ::open(output_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (out_fd < 0) {
    return result;
  }
  std::vector<std::string> argv_storage;
  argv_storage.push_back(exe);
  argv_storage.insert(argv_storage.end(), args.begin(), args.end());
  std::vector<char *> argv;
  for (std::string &a : argv_storage) {
    argv.push_back(a.data());
  }
  argv.push_back(nullptr);

  const pid_t pid = ::fork();
  if (pid < 0) {
    ::close(out_fd);
    return result;
  }
  if (pid == 0) {
    ::setpgid(0, 0); // own group, so the whole tree can be killed
    ::dup2(out_fd, STDOUT_FILENO);
    ::dup2(out_fd, STDERR_FILENO);
    const int null_fd = ::open("/dev/null", O_RDONLY);
    if (null_fd >= 0) {
      ::dup2(null_fd, STDIN_FILENO);
    }
    ::execv(exe.c_str(), argv.data());
    ::_exit(127);
  }
  ::close(out_fd);
  result.started = true;

  const auto start = std::chrono::steady_clock::now();
  int status = 0;
  for (;;) {
    const pid_t done = ::waitpid(pid, &status, WNOHANG);
    if (done == pid) {
      break;
    }
    if (done < 0 && errno != EINTR) {
      return result;
    }
    if (timeout_seconds > 0.0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >
            timeout_seconds) {
      result.timed_out = true;
      ::kill(-pid, SIGKILL);
      ::waitpid(pid, &status, 0);
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (WIFEXITED(status)) {
    result.exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    result.exit_code = 128 + WTERMSIG(status);
    result.crashed = !result.timed_out;
  }
  return result;
}

std::string grim_self_exe_path(const std::string &argv0) {
  char buf[4096];
  const ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (n > 0) {
    return std::string(buf, static_cast<size_t>(n));
  }
  return argv0;
}

#endif
