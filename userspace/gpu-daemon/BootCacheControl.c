// SPDX-License-Identifier: MIT
// TB375FC M7/P2 BootCacheControl shim (arm64, ad-hoc signed, persistence-safe).
// Invoked by macOS launchd before WindowServer starts.
// Spawns /usr/sbin/tb375fc-gpu-daemon detached in the background and waits until
// it publishes /private/var/tmp/tb375fc-gpud.ready (connected to IOAccelSwiftShader),
// then exits 0 so launchd proceeds immediately to WindowServer.
//
// P2 persistence fix: /private/var/tmp is on persistent UFS APFS Data, so a stale
// tb375fc-gpud.ready from a previous boot may exist. Verify that the PID inside is
// actually a live tb375fc-gpu-daemon process; otherwise unlink and launch fresh.
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <libproc.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <unistd.h>

static int gCon = -1;

static void clog(const char *fmt, ...) {
    char buf[384];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if ((size_t)n >= sizeof(buf)) n = (int)sizeof(buf) - 1;
    if (gCon >= 0) (void)write(gCon, buf, (size_t)n);
    (void)write(STDERR_FILENO, buf, (size_t)n);
}

static long long now_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000LL + (long long)tv.tv_usec / 1000LL;
}

static int gpud_is_live(const char *ready_path) {
    FILE *f = fopen(ready_path, "r");
    if (!f) return 0;
    int pid = 0;
    int ok = (fscanf(f, "%d", &pid) == 1);
    fclose(f);
    if (!ok || pid <= 1) return 0;
    if (kill(pid, 0) != 0) return 0;
    char name[64] = {0};
    if (proc_name(pid, name, (uint32_t)sizeof(name)) <= 0) return 0;
    return strcmp(name, "tb375fc-gpu-daemon") == 0;
}

static void prune_dir_files(const char *dir_path, int recursive, uint64_t *freed_bytes, uint32_t *removed) {
    DIR *d = opendir(dir_path);
    if (!d) return;
    struct dirent *de;
    char child[512];
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.' && (de->d_name[1] == '\0' || (de->d_name[1] == '.' && de->d_name[2] == '\0')))
            continue;
        int n = snprintf(child, sizeof(child), "%s/%s", dir_path, de->d_name);
        if (n <= 0 || (size_t)n >= sizeof(child)) continue;
        struct stat st;
        if (lstat(child, &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) {
            if (recursive) prune_dir_files(child, 1, freed_bytes, removed);
        } else if (S_ISREG(st.st_mode)) {
            if (unlink(child) == 0) {
                *freed_bytes += (uint64_t)st.st_size;
                (*removed)++;
            }
        }
    }
    closedir(d);
}

// Keep 1 GiB persistent UFS Data partition from ever hitting ENOSPC after long uptimes (e.g. 12h+ logs).
static void prune_persistent_logs(void) {
    struct statvfs vfs;
    uint64_t free_mb = 9999;
    if (statvfs("/private/var", &vfs) == 0) {
        free_mb = ((uint64_t)vfs.f_bavail * (uint64_t)vfs.f_frsize) >> 20;
    }
    (void)unlink("/private/var/tmp/gpud.log");
    uint64_t freed = 0;
    uint32_t removed = 0;
    // Always prune macOS unified diagnostic logs & crash/analytics logs on boot; preserve JIT/m2v caches.
    prune_dir_files("/private/var/db/diagnostics/Persist", 1, &freed, &removed);
    prune_dir_files("/private/var/db/diagnostics/Special", 1, &freed, &removed);
    prune_dir_files("/private/var/db/diagnostics/Signpost", 1, &freed, &removed);
    prune_dir_files("/private/var/log/com.apple.xpc.launchd", 0, &freed, &removed);
    prune_dir_files("/private/var/log/DiagnosticMessages", 1, &freed, &removed);
    if (free_mb < 256) {
        prune_dir_files("/private/var/log", 0, &freed, &removed);
        prune_dir_files("/private/var/db/uuidtext", 1, &freed, &removed);
    }
    if (removed > 0 || free_mb < 256) {
        clog("[m7-boot] BootCacheControl: pruned %u log files (%llu KiB), free_before=%llu MiB\n",
             removed, (unsigned long long)(freed >> 10), (unsigned long long)free_mb);
    }
}

int main(int argc, char **argv) {
    gCon = open("/dev/console", O_WRONLY | O_NOCTTY);
    const char *arg1 = (argc > 1 && argv[1]) ? argv[1] : "(none)";
    prune_persistent_logs();
    const char *ready_path = "/private/var/tmp/tb375fc-gpud.ready";
    const char *daemon_path = "/usr/sbin/tb375fc-gpu-daemon";

    if (access(ready_path, F_OK) == 0) {
        if (gpud_is_live(ready_path)) {
            clog("[m7-boot] BootCacheControl(%s): gpu-daemon already live\n", arg1);
            return 0;
        }
        clog("[m7-boot] BootCacheControl(%s): clearing stale persistent %s\n", arg1, ready_path);
        (void)unlink(ready_path);
    }

    clog("[m7-boot] BootCacheControl(%s) pid=%d uid=%d: launching %s\n",
         arg1, getpid(), getuid(), daemon_path);

    if (access(daemon_path, X_OK) != 0) {
        clog("[m7-boot] ERROR: %s not executable (errno=%d)\n", daemon_path, errno);
        return 0;
    }

    long long t0 = now_ms();
    pid_t pid = fork();
    if (pid < 0) {
        clog("[m7-boot] ERROR: fork failed errno=%d\n", errno);
        return 0;
    }
    if (pid == 0) {
        (void)setsid();
        int devnull = open("/dev/null", O_RDONLY);
        if (devnull >= 0) {
            dup2(devnull, STDIN_FILENO);
            if (devnull > 2) close(devnull);
        }
        int logfd = open("/private/var/tmp/gpud.log", O_CREAT | O_WRONLY | O_APPEND, 0644);
        if (logfd >= 0) {
            dup2(logfd, STDOUT_FILENO);
            dup2(logfd, STDERR_FILENO);
            if (logfd > 2) close(logfd);
        }
        if (gCon > 2) close(gCon);
        char *const dargv[] = { (char *)daemon_path, NULL };
        char *const denvp[] = {
            (char *)"PATH=/usr/sbin:/usr/bin:/sbin:/bin",
            (char *)"TMPDIR=/private/var/tmp",
            NULL
        };
        execve(daemon_path, dargv, denvp);
        int e = errno;
        int c = open("/dev/console", O_WRONLY | O_NOCTTY);
        if (c >= 0) {
            char msg[160];
            int n = snprintf(msg, sizeof(msg), "[m7-boot] FATAL: execve(%s) failed errno=%d\n", daemon_path, e);
            if (n > 0) (void)write(c, msg, (size_t)n);
            close(c);
        }
        _exit(127);
    }

    // Wait up to 8s for readiness sentinel (or early crash)
    for (int i = 0; i < 80; ++i) {
        if (access(ready_path, F_OK) == 0) {
            clog("[m7-boot] gpu-daemon pid=%d READY in %lldms!\n", pid, now_ms() - t0);
            return 0;
        }
        int st = 0;
        pid_t r = waitpid(pid, &st, WNOHANG);
        if (r == pid) {
            if (WIFEXITED(st)) {
                clog("[m7-boot] ERROR: gpu-daemon pid=%d exited early code=%d after %lldms\n",
                     pid, WEXITSTATUS(st), now_ms() - t0);
            } else if (WIFSIGNALED(st)) {
                clog("[m7-boot] ERROR: gpu-daemon pid=%d killed early sig=%d after %lldms\n",
                     pid, WTERMSIG(st), now_ms() - t0);
            }
            return 0;
        }
        usleep(100000);
    }
    clog("[m7-boot] WARN: gpu-daemon pid=%d still starting after 8000ms; continuing boot\n", pid);
    return 0;
}
