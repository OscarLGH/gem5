/*
 * Copyright 2003-2005 The Regents of The University of Michigan
 * Copyright 2007-2008 The Florida State University
 * Copyright 2009 The University of Edinburgh
 * Copyright 2020 Google Inc.
 * Copyright 2021 IBM Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/power/linux/se_workload.hh"

#include <sys/syscall.h>

#include "arch/power/process.hh"
#include "base/loader/object_file.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "sim/syscall_emul.hh"

namespace gem5
{

namespace
{

class LinuxLoader : public Process::Loader
{
  public:
    Process *
    load(const ProcessParams &params, loader::ObjectFile *obj) override
    {
        auto arch = obj->getArch();

        if (arch != loader::Power && arch != loader::Power64)
            return nullptr;

        auto opsys = obj->getOpSys();

        if (opsys == loader::UnknownOpSys) {
            warn("Unknown operating system; assuming Linux.");
            opsys = loader::Linux;
        }

        if ((arch == loader::Power && opsys != loader::Linux) ||
            (arch == loader::Power64 &&
             opsys != loader::LinuxPower64ABIv1 &&
             opsys != loader::LinuxPower64ABIv2))
            return nullptr;

        return new PowerProcess(params, obj);
    }
};

LinuxLoader linuxLoader;

} // anonymous namespace

namespace PowerISA
{

void
EmuLinux::syscall(ThreadContext *tc)
{
    Process *process = tc->getProcessPtr();
    // Call the syscall function in the base Process class to update stats.
    // This will move into the base SEWorkload function at some point.
    process->Process::syscall(tc);

    syscallDescs.get(tc->readIntReg(0))->doSyscall(tc);
}

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<Linux::utsname> name)
{
    auto process = tc->getProcessPtr();

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "sim.gem5.org");
    strcpy(name->release, process->release.c_str());
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "power");

    return 0;
}

SyscallDescTable<PowerISA::SEWorkload::SyscallABI> EmuLinux::syscallDescs = {
    {  0, "restart_syscall" },
    {  1, "exit", exitFunc },
    {  2, "fork" },
    {  3, "read", readFunc<PowerLinux> },
    {  4, "write", writeFunc<PowerLinux> },
    {  5, "open", openFunc<PowerLinux> },
    {  6, "close", closeFunc },
    {  7, "waitpid" }, //???
    {  8, "creat" },
    {  9, "link" },
    { 10, "unlink", unlinkFunc },
    { 11, "execve" },
    { 12, "chdir", chdirFunc },
    { 13, "time", timeFunc<PowerLinux> },
    { 14, "mknod" },
    { 15, "chmod", chmodFunc<PowerLinux> },
    { 16, "lchown", chownFunc },
    { 17, "ni_syscall" }, //???
    { 18, "ni_syscall" }, //???
    { 19, "lseek", lseekFunc },
    { 20, "getpid", getpidFunc },
    { 21, "mount" },
    { 22, "ni_syscall" },
    { 23, "setuid", ignoreFunc },
    { 24, "getuid", getuidFunc },
    { 25, "stime" },
    { 26, "ptrace" },
    { 27, "alarm" },
    { 28, "ni_syscall" },
    { 29, "pause" },
    { 30, "utime" },
    { 31, "ni_syscall" },
    { 32, "ni_syscall" },
    { 33, "access", accessFunc },
    { 34, "nice" },
    { 35, "ni_syscall" },
    { 36, "sync" },
    { 37, "kill", ignoreFunc },
    { 38, "rename", renameFunc },
    { 39, "mkdir" , mkdirFunc },
    { 40, "rmdir" , rmdirFunc },
    { 41, "dup", dupFunc },
    { 42, "pipe" },
    { 43, "times", timesFunc<PowerLinux> },
    { 44, "ni_syscall" },
    { 45, "brk", brkFunc },
    { 46, "setgid" },
    { 47, "getgid", getgidFunc },
    { 48, "signal", ignoreFunc },
    { 49, "geteuid", geteuidFunc },
    { 50, "getegid", getegidFunc },
    { 51, "acct" },
    { 52, "umount" },
    { 53, "ni_syscall" },
    { 54, "ioctl", ioctlFunc<PowerLinux> },
    { 55, "fcntl", fcntlFunc },
    { 56, "ni_syscall" },
    { 57, "setpgid" },
    { 58, "ni_syscall" },
    { 59, "ni_syscall" },
    { 60, "umask", umaskFunc },
    { 61, "chroot" },
    { 62, "ustat" },
    { 63, "dup2" },
    { 64, "getppid", getppidFunc },
    { 65, "getpgrp" },
    { 66, "setsid" },
    { 67, "ni_syscall" },
    { 68, "sgetmask" },
    { 69, "ssetmask" },
    { 70, "setreuid" },
    { 71, "setregid" },
    { 72, "ni_syscall" },
    { 73, "ni_syscall" },
    { 74, "sethostname", ignoreFunc },
    { 75, "setrlimit", ignoreFunc },
    { 76, "ni_syscall" },
    { 77, "getrusage", ignoreFunc },
    { 78, "gettimeofday" ,gettimeofdayFunc<PowerLinux>},
    { 79, "settimeofday" },
    { 80, "getgroups" },
    { 81, "setgroups" },
    { 82, "ni_syscall" },
    { 83, "symlink" },
    { 84, "ni_syscall" },
    { 85, "readlink", readlinkFunc },
    { 86, "uselib" },
    { 87, "swapon" },
    { 88, "reboot" },
    { 89, "ni_syscall" },
    { 90, "mmap", mmapFunc<PowerLinux> },
    { 91, "munmap",munmapFunc },
    { 92, "truncate", truncateFunc },
    { 93, "ftruncate", ftruncateFunc },
    { 94, "fchmod" },
    { 95, "fchown" },
    { 96, "getpriority" },
    { 97, "setpriority" },
    { 98, "ni_syscall" },
    { 99, "statfs" },
    { 100, "fstatfs" },
    { 101, "ni_syscall" },
    { 102, "socketcall" },
    { 103, "syslog" },
    { 104, "setitimer" },
    { 105, "getitimer" },
    { 106, "newstat",  statFunc<PowerLinux> },
    { 107, "newlstat" , statFunc<PowerLinux> },
    { 108, "newfstat", fstatFunc<PowerLinux> },
    { 109, "ni_syscall" },
    { 110, "ni_syscall" },
    { 111, "vhangup" },
    { 112, "ni_syscall", ignoreFunc },
    { 113, "ni_syscall" },
    { 114, "wait4" },
    { 115, "swapoff" },
    { 116, "sysinfo", sysinfoFunc<PowerLinux> },
    { 117, "ipc" },
    { 118, "fsync" },
    { 119, "ni_syscall" },
    { 120, "clone" },
    { 121, "setdomainname" },
    { 122, "newuname", unameFunc },
    { 123, "ni_syscall" },
    { 124, "adjtimex" },
    { 125, "mprotect", ignoreFunc },
    { 126, "ni_syscall" },
    { 127, "ni_syscall" },
    { 128, "init_module" },
    { 129, "delete_module" },
    { 130, "ni_syscall" },
    { 131, "quotactl" },
    { 132, "getpgid" },
    { 133, "fchdir" },
    { 134, "bdflush" },
    { 135, "sysfs" },
    { 136, "personality" },
    { 137, "ni_syscall" },
    { 138, "setfsuid" },
    { 139, "setfsgid" },
    { 140, "llseek", _llseekFunc },
    { 141, "getdents", getdentsFunc },
    { 142, "select" },
    { 143, "flock" },
    { 144, "msync" },
    { 145, "readv" },
    { 146, "writev", writevFunc<PowerLinux> },
    { 147, "getsid" },
    { 148, "fdatasync" },
    { 149, "ni_syscall" },
    { 150, "mlock" },
    { 151, "munlock" },
    { 152, "mlockall" },
    { 153, "munlockall" },
    { 154, "sched_setparam" },
    { 155, "sched_getparam" },
    { 156, "sched_setscheduler" },
    { 157, "sched_getscheduler" },
    { 158, "sched_yield" },
    { 159, "sched_get_priority_max" },
    { 160, "sched_get_priority_min" },
    { 161, "sched_rr_get_interval" },
    { 162, "nanosleep" },
    { 163, "mremap" , mremapFunc<PowerLinux> },
    { 164, "setresuid" },
    { 165, "getresuid" },
    { 166, "ni_syscall" },
    { 167, "poll" },
    { 168, "ni_syscall" },
    { 169, "setresgid" },
    { 170, "getresgid" },
    { 171, "prctl" },
    { 172, "rt_sigreturn" },
    { 173, "rt_sigaction", ignoreFunc },
    { 174, "rt_sigprocmask", ignoreFunc},
    { 175, "rt_sigpending" },
    { 176, "rt_sigtimedwait" },
    { 177, "rt_sigqueueinfo", ignoreFunc },
    { 178, "rt_sigsuspend" },
    { 179, "pread64" },
    { 180, "pwrite64" },
    { 181, "chown" },
    { 182, "getcwd", getcwdFunc },
    { 183, "capget" },
    { 184, "capset" },
    { 185, "sigaltstack" },
    { 186, "sendfile" },
    { 187, "ni_syscall" },
    { 188, "ni_syscall" },
    { 189, "vfork" },
    { 190, "getrlimit", ignoreFunc },
    { 191, "readahead" },
    { 192, "ni_syscall"},
    { 193, "ni_syscall" },
    { 194, "ni_syscall" },
    { 195, "ni_syscall" },
    { 196, "ni_syscall" },
    { 197, "ni_syscall" },
    { 198, "pciconfigread" },
    { 199, "pciconfigwrite" },
    { 200, "pciconfig_iobase" },
    { 201, "ni_syscall" },
    { 202, "getdents64", getdents64Func},
    { 203, "pivot_root" },
    { 204, "ni_syscall", },
    { 205, "madvise" },
    { 206, "mincore" },
    { 207, "gettid", gettidFunc },
    { 208, "tkill" },
    { 209, "setxattr" },
    { 210, "lsetxattr" },
    { 211, "fsetxattr" },
    { 212, "chown" },
    { 213, "setuid" },
    { 214, "setgid" },
    { 215, "setfsuid" },
    { 216, "setfsgid" },
    { 217, "getdents64" },
    { 218, "pivot_root" },
    { 219, "mincore" },
    { 220, "madvise" },
    { 221, "futex", futexFunc<PowerLinux> },
    { 222, "sched_setaffinity", ignoreFunc },
    { 223, "sched_getaffinity", schedGetaffinityFunc<PowerLinux> },
    { 224, "gettid" },
    { 225, "readahead" },
    { 226, "setxattr" },
    { 227, "lsetxattr" },
    { 228, "fsetxattr" },
    { 229, "getxattr" },
    { 230, "lgetxattr" },
    { 231, "fgetxattr" },
    { 232, "listxattr" , listxattrFunc },
    { 233, "llistxattr" },
    { 234, "exit_group", exitGroupFunc },
    { 235, "removexattr" },
    { 236, "lremovexattr" },
    { 237, "fremovexattr" },
    { 238, "tkill" },
    { 239, "sendfile64" },
    { 240, "timer_create" },
    { 241, "timer_settime" },
    { 242, "timer_gettime" },
    { 243, "timer_getoverrun" , ignoreFunc },
    { 244, "timer_delete", ignoreFunc },
    { 245, "timer_delete", ignoreFunc },
    { 246, "clock_gettime", clock_gettimeFunc<PowerLinux> },
    { 247, "clock_getres", clock_getresFunc<PowerLinux> },
    { 248, "clock_nanosleep" },
    { 249, "sys_swapcontext" },
    { 250, "tgkill", tgkillFunc<PowerLinux> },
    { 251, "utimes" },
    { 252, "statfs64" },
    { 253, "fstatfs64" },
    { 254, "ni_syscall" },
    { 255, "rtas" },
    { 256, "ni_syscall" },
    { 257, "ni_syscall" },
    { 258, "migrate_pages" },
    { 259, "mbind" },
    { 260, "get_mempolicy" },
    { 261, "set_mempolicy" },
    { 262, "mq_open" },
    { 263, "mq_unlink" },
    { 264, "mq_timedsend" },
    { 265, "sys_mq_timedreceive" },
    { 266, "mq_notify" },
    { 267, "mq_getsetattr" },
    { 268, "kexec_load" },
    { 269, "add_key" },
    { 270, "request_key" },
    { 271, "keyctl" },
    { 272, "waitid" },
    { 273, "ioprio_set" },
    { 274, "ioprio_get" },
    { 275, "inotify_init" },
    { 276, "inotify_add_watch" },
    { 277, "inotify_rm_watch" },
    { 278, "spu_run" },
    { 279, "spu_create" },
    { 280, "pselect6" },
    { 281, "ppoll" },
    { 282, "unshare" },
    { 283, "splice" },
    { 284, "tee" },
    { 285, "vmsplice" },
    { 286, "openat", openatFunc<PowerLinux>},
    { 287, "mkdirat" },
    { 288, "mknodat" },
    { 289, "fchownat" },
    { 290, "futimesat" },
    { 291, "newfstatat" , fstatatFunc<PowerLinux>},
    { 292, "unlinkat" },
    { 293, "renameat" },
    { 294, "linkat" },
    { 295, "symlinkat" },
    { 296, "readlinkat" },
    { 297, "fchmodat" },
    { 298, "faccessat" },
    { 299, "get_robust_list" },
    { 300, "set_robust_list", ignoreFunc},
    { 301, "move_pages" },
    { 302, "getcpu" },
    { 303, "epoll_pwait" },
    { 304, "utimensat" },
    { 305, "signalfd" },
    { 306, "timerfd_create" },
    { 307, "eventfd" },
    { 308, "sync_file_range2" },
    { 309, "fallocate" },
    { 310, "subpage_prot" },
    { 311, "timerfd_settime" },
    { 312, "timerfd_gettime" },
    { 313, "signalfd4" },
    { 314, "eventfd2" },
    { 315, "epoll_create1" },
    { 316, "dup3" },
    { 317, "pipe2" },
    { 318, "inotify_init1" },
    { 319, "perf_event_open" },
    { 320, "preadv" },
    { 321, "pwritev" },
    { 322, "rt_tgsigqueueinfo" },
    { 323, "fanotify_init" },
    { 324, "fanotify_mark" },
    { 325, "prlimit64" , prlimitFunc<PowerLinux> },
    { 326, "socket" },
    { 327, "bind" },
    { 328, "connect" },
    { 329, "listen" },
    { 330, "accept" },
    { 331, "getsockname" },
    { 332, "getpeername" },
    { 333, "socketpair" },
    { 334, "send" },
    { 335, "sendto" },
    { 336, "recv" },
    { 337, "recvfrom" },
    { 338, "shutdown" },
    { 339, "setsockopt" },
    { 340, "getsockopt" },
    { 341, "sys_sendmsg" },
    { 342, "recvmsg" },
    { 343, "recvmmsg" },
    { 344, "accept4" },
    { 345, "name_to_handle_at" },
    { 346, "open_by_handle_at" },
    { 347, "sys_clock_adjtime" },
    { 348, "sys_syncfs" },
    { 349, "sys_sendmmsg" },
    { 350, "sys_setns" },
    { 351, "sys_process_vm_readv" },
    { 352, "sys_process_vm_writev" },
    { 353, "sys_finit_module" },
    { 354, "sys_kcmp" },
    { 355, "sys_sched_setattr" },
    { 356, "sys_sched_getattr" },
    { 357, "sys_renameat2" },
    { 358, "sys_seccomp" },
    { 359, "getrandom", ignoreFunc },
    { 360, "sys_memfd_create " },
    { 361, "sys_bpf " },
    { 362, "sys_execveat " },
    { 363, "sys_switch_endian " },
    { 364, "sys_userfaultfd " },
    { 365, "sys_membarrier " },
    { 366, "sys_ni_syscall " },
    { 367, "sys_ni_syscall " },
    { 368, "sys_ni_syscall " },
    { 369, "sys_ni_syscall " },
    { 370, "sys_ni_syscall " },
    { 371, "sys_ni_syscall " },
    { 372, "sys_ni_syscall " },
    { 373, "sys_ni_syscall " },
    { 374, "sys_ni_syscall " },
    { 375, "sys_ni_syscall " },
    { 376, "sys_ni_syscall " },
    { 377, "sys_ni_syscall " },
    { 378, "sys_mlock2 " },
    { 379, "sys_copy_file_range " },
    { 380, "sys_preadv2 " },
    { 381, "sys_pwritev2 " },
    { 382, "sys_kexec_file_load " },
    { 383, "sys_statx " },
    { 384, "sys_pkey_alloc " },
    { 385, "sys_pkey_free " },
    { 386, "sys_pkey_mprotect " },
    { 387, "sys_rseq " },
    { 388, "sys_io_pgetevents " },
    { 389, "sys_ni_syscall " },
    { 390, "sys_ni_syscall " },
    { 391, "sys_ni_syscall " },
    { 392, "sys_semtimedop " },
    { 393, "sys_semget " },
    { 394, "sys_semctl " },
    { 395, "sys_shmget " },
    { 396, "sys_shmctl " },
    { 397, "sys_shmat " },
    { 398, "sys_shmdt " },
    { 399, "sys_msgget " },
    { 400, "sys_msgsnd " },
    { 401, "sys_msgrcv " },
    { 402, "sys_msgctl " },
    { 403, "sys_ni_syscall " },
    { 404, "sys_ni_syscall " },
    { 405, "sys_ni_syscall " },
    { 406, "sys_ni_syscall " },
    { 407, "sys_ni_syscall " },
    { 408, "sys_ni_syscall " },
    { 409, "sys_ni_syscall " },
    { 410, "sys_ni_syscall " },
    { 411, "sys_ni_syscall " },
    { 412, "sys_ni_syscall " },
    { 413, "sys_ni_syscall " },
    { 414, "sys_ni_syscall " },
    { 415, "sys_ni_syscall " },
    { 416, "sys_ni_syscall " },
    { 417, "sys_ni_syscall " },
    { 418, "sys_ni_syscall " },
    { 419, "sys_ni_syscall " },
    { 420, "sys_ni_syscall " },
    { 421, "sys_ni_syscall " },
    { 422, "sys_ni_syscall " },
    { 423, "sys_ni_syscall " },
    { 424, "sys_pidfd_send_signal " },
    { 425, "sys_io_uring_setup " },
    { 426, "sys_io_uring_enter " },
    { 427, "sys_io_uring_register " },
    { 428, "sys_open_tree " },
    { 429, "sys_move_mount " },
    { 430, "sys_fsopen " },
    { 431, "sys_fsconfig " },
    { 432, "sys_fsmount " },
    { 433, "sys_fspick " },
    { 434, "sys_pidfd_open " },
    { 435, "sys_clone3 " },
    { 436, "sys_close_range " },
    { 437, "sys_openat2 " },
    { 438, "sys_pidfd_getfd " },
    { 439, "sys_faccessat2 " },
    { 440, "sys_process_madvise " },
    { 441, "sys_epoll_pwait2 " },
    { 442, "sys_mount_setattr " }
};

} // namespace PowerISA
} // namespace gem5
