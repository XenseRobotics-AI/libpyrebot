/* Test-only SocketCAN replacement. Routes native can_frame reads/writes to a
 * Unix packet socket; the test process never opens a physical CAN interface. */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

static int test_fd = -1;
static unsigned int failed_writes = 0;
static unsigned int enabled_motors = 0;

unsigned int rebot_test_failed_writes(void) { return failed_writes; }

ssize_t write(int fd, const void *buffer, size_t count) {
  ssize_t (*real_write)(int, const void *, size_t) = dlsym(RTLD_NEXT, "write");
  if (fd == test_fd && count == sizeof(struct can_frame)) {
    const struct can_frame *frame = buffer;
    const unsigned int kind = (frame->can_id >> 24) & 0x1F;
    const char *failure = getenv("REBOT_TEST_CAN_FAILURE");
    const int position = kind == 18 && frame->data[0] == 0x16 && frame->data[1] == 0x70;
    if (position && failure) {
      const int persistent = !strcmp(failure, "persistent");
      const int down = !strcmp(failure, "down");
      const int startup = !strcmp(failure, "startup") && enabled_motors == 0;
      const int runtime = !strcmp(failure, "runtime") && enabled_motors >= 6;
      if (persistent || down || ((startup || runtime) && failed_writes < 3)) {
        const int transient_errors[] = {ENOBUFS, EAGAIN, EINTR};
        errno = down ? ENETDOWN : (persistent ? ENOBUFS : transient_errors[failed_writes]);
        ++failed_writes;
        return -1;
      }
    }
    ssize_t sent = real_write(fd, buffer, count);
    if (sent == (ssize_t)count && kind == 3) ++enabled_motors;
    return sent;
  }
  return real_write(fd, buffer, count);
}

int socket(int domain, int type, int protocol) {
  int (*real_socket)(int, int, int) = dlsym(RTLD_NEXT, "socket");
  if (domain != AF_CAN) return real_socket(domain, type, protocol);
  const char *path = getenv("REBOT_TEST_CAN_SOCKET");
  if (!path || strlen(path) >= sizeof(((struct sockaddr_un *)0)->sun_path)) {
    errno = EINVAL;
    return -1;
  }
  int fd = real_socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
  if (fd < 0) return fd;
  struct sockaddr_un addr = {.sun_family = AF_UNIX};
  strcpy(addr.sun_path, path);
  if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    close(fd);
    return -1;
  }
  /* The production code expects non-blocking reads. */
  int flags;
  flags = fcntl(fd, F_GETFL);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  test_fd = fd;
  return fd;
}

unsigned int if_nametoindex(const char *name) {
  if (!strcmp(name, "_rebot_test_can")) return 1;
  unsigned int (*real_fn)(const char *) = dlsym(RTLD_NEXT, "if_nametoindex");
  return real_fn(name);
}

int bind(int fd, const struct sockaddr *addr, socklen_t len) {
  if (fd == test_fd && addr->sa_family == AF_CAN) return 0;
  int (*real_bind)(int, const struct sockaddr *, socklen_t) = dlsym(RTLD_NEXT, "bind");
  return real_bind(fd, addr, len);
}
